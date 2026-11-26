// Bench test for a Seeed XIAO ESP32-C5 (also builds for the XIAO ESP32-S3): FEETECH STS3215 12 V bus servos and
// a WS2812B ring. Same commands as the Arduino Nano version in ../../arduino-nano/sts3215_led_test.
//
// Wiring: servo data (one wire, servos daisy-chained) on D2, ring DIN on D3, all grounds common.
// XIAO ESP32-C5: D2 = GPIO25, D3 = GPIO7. Neither is a boot-strapping pin (C5 straps: GPIO2, 3, 26, 27, 28).
// Power the board from USB OR from the buck on the 5V pad, never both: a C5 on both died on 2026-11-24.
//
// The servo bus is one half-duplex wire on D2 at 1 Mbit/s through UART1. The pad is push-pull while sending
// and open drain with pull-up while listening. Open drain alone did not work (the servo never answered).
// Our own bytes echo back and are skipped.
//
// Commands over USB serial (115200, newline terminated). Nothing moves on boot.
//   scan [maxid]                 ping IDs 0..maxid (default 20). Read only.
//   swing <id> <deg> [tq%]       +deg, back, -deg, back. |deg| <= 180. Torque cap default 30 %.
//   go <id> <deg> [tq%]          one move by deg (sign = direction), then back.
//   seq <id> <p1> [p2]           guarded moves to absolute positions.
//   dock [speed] [trip]          ID 2 out to the dock position, then hold (torque on, 30 % cap).
//   undock [speed] [trip]        ID 2 back toward retracted, then torque off.
//   push <deg> [speed] [cap%] [trip] [stall_ms]   careful move of ID 2, then hold at <= 30 %. Defaults 30 / 20 % / 100 / 400.
//   endure [sec] [n]             test motion over and over with the rainbow, one line per cycle (0 = until 's').
//   arm <sec> | disarm | report  one-shot endurance run at the next power-up (for buck power), stats kept in flash.
//   rainbow | rainbow <n> <sec> | off    LEDs on D3 (rainbow runs by default).
//   raw <id> | line | drive <0|1>       bus debugging.
//   relax <id>                   torque off, cap back to 100 %.
//   move <id> <deg> [speed] [cap%] [trip] [stall_ms]   guarded move of any servo, then hold at <= 30 %.
//   stat <id>                    one status line for the UI.
//   's' during any move or endurance run stops it and holds the joint where it is.
//   temp                         chip temperature.
//   ledpin <gpio> | pinhigh <gpio>      ring diagnostics: move the ring data pin, or hold a pin at 3.3 V.
// Every move caps torque (register 48, RAM), holds the current position before enabling torque, and stops
// and holds where it is if the load reaches the trip (default 80 % of the cap) or the joint stops for 0.4 s.
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_reg.h>
#include <esp_rom_gpio.h>
#include <hal/uart_ll.h>

static const int BUS_PIN = D2;
static int RING_PIN = D3;
static const uart_port_t BUS = UART_NUM_1;
static bool ppTx = true;  // push-pull while sending

static Adafruit_NeoPixel ring(16, RING_PIN, NEO_GRB + NEO_KHZ800);

enum : uint8_t {
  R_MIN_LIM = 9, R_MAX_LIM = 11, R_MODE = 33, R_TORQUE = 40, R_ACC = 41, R_GOAL = 42,
  R_TQ_LIMIT = 48, R_POS = 56, R_LOAD = 60, R_VOLT = 62, R_TEMP = 63, R_CURRENT = 69
};

static void say(const char *fmt, ...) {
  char b[200];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  Serial.println(b);
}
#define SAY(fmt, ...) say(fmt, ##__VA_ARGS__)

// steps -> "123.4"
static char *deg(int32_t steps, char *b) { dtostrf(steps * 360.0 / 4096, 0, 1, b); return b; }

// ---------------------------------------------------------------- servo bus
static bool busOpen(int pin, int baud) {
  if (uart_is_driver_installed(BUS)) uart_driver_delete(BUS);
  uart_config_t c = {};
  c.baud_rate = baud;
  c.data_bits = UART_DATA_8_BITS;
  c.parity = UART_PARITY_DISABLE;
  c.stop_bits = UART_STOP_BITS_1;
  c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  c.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(BUS, 512, 0, 0, NULL, 0) != ESP_OK) return false;
  if (uart_param_config(BUS, &c) != ESP_OK) return false;
  if (uart_set_pin(BUS, pin, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
  // uart_set_pin leaves a shared pad input-only. Re-enable output as open drain, then route both signals.
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT_OD);
  esp_rom_gpio_connect_out_signal(pin, U1TXD_OUT_IDX, false, false);
  esp_rom_gpio_connect_in_signal(pin, U1RXD_IN_IDX, false);
  gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLUP_ONLY);
  return true;
}

static void busSend(const uint8_t *pkt, int k) {
  uint32_t reg = GPIO_PIN0_REG + 4 * BUS_PIN;
  if (ppTx) REG_CLR_BIT(reg, BIT(GPIO_PIN0_PAD_DRIVER_S));  // push-pull
  uart_write_bytes(BUS, pkt, k);
  if (ppTx) {
    uint32_t t0 = micros();
    while (!uart_ll_is_tx_idle(UART_LL_GET_HW(BUS)) && micros() - t0 < 20000) {}
    REG_SET_BIT(reg, BIT(GPIO_PIN0_PAD_DRIVER_S));  // back to open drain before the servo answers
  }
  uart_wait_tx_done(BUS, pdMS_TO_TICKS(10));
}

static uint32_t g_commErr = 0;  // replies missing or bad (endurance stat)
static uint8_t lastRaw[128];
static int lastRawN = 0;

// Sends one packet. Returns number of reply params (>= 0), or < 0: -1 silence, -2 garbage, -3 short, -4 checksum.
static int txrx(uint8_t id, uint8_t ins, const uint8_t *p, int n, uint8_t *out, int outLen, uint8_t *errOut = nullptr) {
  uint8_t pkt[48];
  int k = 0;
  pkt[k++] = 0xFF; pkt[k++] = 0xFF; pkt[k++] = id; pkt[k++] = n + 2; pkt[k++] = ins;
  uint8_t sum = id + (n + 2) + ins;
  for (int i = 0; i < n; i++) { pkt[k++] = p[i]; sum += p[i]; }
  pkt[k++] = ~sum;
  uart_flush_input(BUS);
  busSend(pkt, k);
  int want = k + 6 + outLen, got = 0;
  uint32_t t0 = millis();
  while (got < want && millis() - t0 < 6) {
    int r = uart_read_bytes(BUS, lastRaw + got, want - got, pdMS_TO_TICKS(1));
    if (r > 0) got += r;
  }
  lastRawN = got;
  int start = (got >= k && memcmp(lastRaw, pkt, k) == 0) ? k : 0;  // skip our own echo
  if (id == 0xFE) return 0;
  for (int i = start; i + 5 < got; i++) {
    if (lastRaw[i] != 0xFF || lastRaw[i + 1] != 0xFF || lastRaw[i + 2] != id) continue;
    int len = lastRaw[i + 3];
    if (i + 3 + len >= got) { g_commErr++; return -3; }
    uint8_t s = 0;
    for (int j = 2; j < 3 + len; j++) s += lastRaw[i + j];
    if ((uint8_t)~s != lastRaw[i + 3 + len]) { g_commErr++; return -4; }
    if (errOut) *errOut = lastRaw[i + 4];
    int np = len - 2;
    for (int j = 0; j < np && j < outLen; j++) out[j] = lastRaw[i + 5 + j];
    return np;
  }
  g_commErr++;
  return got > start ? -2 : -1;
}

static bool rd(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out) {
  uint8_t p[2] = {addr, len};
  return txrx(id, 0x02, p, 2, out, len) == len;
}
static bool rd16(uint8_t id, uint8_t addr, int *v) {
  uint8_t b[2];
  if (!rd(id, addr, 2, b)) return false;
  *v = b[0] | (b[1] << 8);
  return true;
}
static bool rd8(uint8_t id, uint8_t addr, int *v) {
  uint8_t b[1];
  if (!rd(id, addr, 1, b)) return false;
  *v = b[0];
  return true;
}
static bool wr(uint8_t id, uint8_t addr, const uint8_t *d, uint8_t n) {
  uint8_t p[16];
  p[0] = addr;
  memcpy(p + 1, d, n);
  uint8_t dummy[1];
  return txrx(id, 0x03, p, n + 1, dummy, 0) >= 0;
}
static bool wr8(uint8_t id, uint8_t addr, uint8_t v) { return wr(id, addr, &v, 1); }
static bool wr16(uint8_t id, uint8_t addr, int v) {
  uint8_t d[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
  return wr(id, addr, d, 2);
}
static bool wrGoal(uint8_t id, int goal, int speed, int acc) {
  // 41 acc, 42-43 goal, 44-45 time, 46-47 speed
  uint8_t d[7] = {(uint8_t)acc, (uint8_t)(goal & 0xFF), (uint8_t)(goal >> 8), 0, 0,
                  (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)};
  return wr(id, R_ACC, d, 7);
}
static int loadSigned(int raw) { return (raw & 0x400) ? -(raw & 0x3FF) : (raw & 0x3FF); }

// ---------------------------------------------------------------- commands
static void cmdScan(int maxId) {
  int n = 0;
  char b1[10];
  for (int id = 0; id <= maxId; id++) {
    uint8_t dummy[1], err = 0;
    int8_t r = txrx(id, 0x01, nullptr, 0, dummy, 0, &err);
    if (r == -2 || r == -3 || r == -4) SAY("  id %d: bad reply (%d)", id, r);
    if (r < 0) continue;
    n++;
    int pos = -1, mode = -1, tq = -1, v = -1, t = -1, mn = -1, mx = -1;
    rd16(id, R_POS, &pos); rd8(id, R_MODE, &mode); rd8(id, R_TORQUE, &tq);
    rd8(id, R_VOLT, &v); rd8(id, R_TEMP, &t); rd16(id, R_MIN_LIM, &mn); rd16(id, R_MAX_LIM, &mx);
    SAY("  FOUND id %d: pos %d (%s deg) mode %d torque %d  %d.%d V  %d C  limits %d..%d  err 0x%02X", id, pos,
        deg(pos, b1), mode, tq, v / 10, v % 10, t, mn, mx, err);
  }
  SAY("scan done: %d servo(s)", n);
}

static int g_tqLimit = 300;
static int g_tripOverride = 0;
static uint32_t g_stallMs = 400;  // stop if the joint has not moved for this long   // > 0: trip at this load instead of 80 % of the cap
static bool g_quiet = false;     // endurance: no per-move prints
static int g_runPeak = 0;        // largest |load| seen in the last runMoves
static int g_runWorstErr = 0;    // largest |final error| in steps over the moves that reached
static bool g_anim = false;      // animate the rainbow while moving
static bool g_stopReq = false;   // 's' from the PC
static uint16_t g_hue = 0;
static uint32_t g_lastFrame = 0;
static bool g_rainbow = true;      // rainbow at idle and during moves ("off" stops it, "rainbow" starts it)
static uint32_t g_quietUntil = 0;  // no LED frames until then: a command is coming in over serial

static void ringTick() {
  if (!(g_anim || g_rainbow) || (int32_t)(millis() - g_quietUntil) < 0 || millis() - g_lastFrame < 30) return;
  g_lastFrame = millis();
  ring.rainbow(g_hue);
  ring.show();  // RMT driven on the ESP32
  g_hue += 384;
}
static void pollStop() {
  while (Serial.available()) if (Serial.read() == 's') g_stopReq = true;
}
static void wait(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { ringTick(); pollStop(); }
}

// 0 = reached, 1 = blocked, 2 = timeout
static int moveGuarded(uint8_t id, int goal, int speed, int acc, uint32_t ms) {
  char b1[10];
  wrGoal(id, goal, speed, acc);
  const int loadTrip = g_tripOverride > 0 ? g_tripOverride : g_tqLimit * 8 / 10;
  uint32_t t0 = millis(), lastProg = millis(), lastPrint = 0;
  int pos = -1, lastPos = -1, load = 0, cur = 0, peak = 0, raw;
  while (millis() - t0 < ms) {
    if (!rd16(id, R_POS, &pos)) { wait(10); if (g_stopReq) break; continue; }
    if (g_stopReq) {  // 's' from the PC: stop and hold where it is
      wrGoal(id, pos, speed, acc);
      SAY("    id %d STOPPED by PC at %d", id, pos);
      return 1;
    }
    if (rd16(id, R_LOAD, &raw)) load = loadSigned(raw);
    rd16(id, R_CURRENT, &cur);
    if (abs(load) > abs(peak)) peak = load;
    if (abs(load) > g_runPeak) g_runPeak = abs(load);
    if (!g_quiet && millis() - lastPrint >= 300) {
      lastPrint = millis();
      SAY("      t=%4lums pos %d (%s deg to go) load %d cur %d", millis() - t0, pos, deg(goal - pos, b1), load, cur);
    }
    if (abs(pos - goal) <= 8) {
      if (abs(pos - goal) > g_runWorstErr) g_runWorstErr = abs(pos - goal);
      if (!g_quiet) SAY("    reached %d (%s deg off), peak load %d", pos, deg(pos - goal, b1), peak);
      return 0;
    }
    if (lastPos < 0 || abs(pos - lastPos) >= 3) { lastPos = pos; lastProg = millis(); }
    bool tooHard = abs(load) >= loadTrip;
    bool stuck = millis() - t0 > 300 && millis() - lastProg > g_stallMs;
    if (tooHard || stuck) {
      wrGoal(id, pos, speed, acc);  // stop pushing, hold here
      SAY("    id %d BLOCKED at %d (%s deg short): %s, load %d (trip %d)", id, pos, deg(abs(goal - pos), b1),
          tooHard ? "load over trip" : "stopped moving", load, loadTrip);
      return 1;
    }
    wait(15);
  }
  wrGoal(id, pos >= 0 ? pos : goal, speed, acc);
  SAY("    id %d TIMEOUT at %d", id, pos);
  return 2;
}

// Moves id by each of the offsets in degs[] (relative to the start), returning to start after each.
// Returns 0 ok, 1 an outward move was blocked or timed out, 2 could not get back to start, 3 read/limit failure.
// home >= 0: moves are relative to home instead of to where the servo is (endurance: no drift over many cycles).
static int runMoves(int id, const int *degs, uint8_t nMoves, int tqPct, int home = -1) {
  int status = 0;
  g_runPeak = 0; g_runWorstErr = 0;
  char b1[10];
  if (tqPct < 5 || tqPct > 100) { SAY("torque %% must be 5..100"); return 3; }
  g_tqLimit = tqPct * 10;
  const int SPEED = 150, ACC = 10;  // ~13 deg/s, gentle accel
  int pos0, mode, tq, mn, mx, tl0 = 1000;
  if (!rd16(id, R_POS, &pos0) || !rd8(id, R_MODE, &mode) || !rd8(id, R_TORQUE, &tq) ||
      !rd16(id, R_MIN_LIM, &mn) || !rd16(id, R_MAX_LIM, &mx)) { SAY("id %d: read failed", id); return 3; }
  if (mode != 0) { SAY("id %d: mode %d is not position mode", id, mode); return 3; }
  int lo = (mx > mn) ? mn : 0, hi = (mx > mn) ? mx : 4095;
  int posNow = pos0;
  if (home >= 0) {
    if (abs(home - pos0) > 60) { SAY("id %d: %d is more than 5 deg from home %d", id, pos0, home); return 2; }
    pos0 = home;
  }
  for (uint8_t i = 0; i < nMoves; i++) {
    long g = pos0 + lroundf(degs[i] * 4096.0f / 360.0f);
    if (g < lo || g > hi) { SAY("id %d: %+d deg -> %ld is outside limits %d..%d", id, degs[i], g, lo, hi); return 3; }
  }
  rd16(id, R_TQ_LIMIT, &tl0);
  if (!g_quiet)
    SAY("id %d: start %d (%s deg), torque was %d, torque cap %d%% (was %d%%)", id, pos0, deg(pos0, b1), tq, tqPct,
        tl0 / 10);
  wr16(id, R_TQ_LIMIT, g_tqLimit);
  wrGoal(id, posNow, SPEED, ACC);  // hold where it is before torque on, so it does not jump
  if (tq == 0) { wr8(id, R_TORQUE, 1); wait(50); }
  if (posNow != pos0 && moveGuarded(id, pos0, SPEED, ACC, 2000) != 0) status = 2;
  for (uint8_t i = 0; i < nMoves && status < 2; i++) {
    int d = lroundf(degs[i] * 4096.0f / 360.0f);
    uint32_t ms = 1500 + 2000UL * abs(d) / SPEED;
    if (!g_quiet) SAY("  %+d deg -> %d", degs[i], pos0 + d);
    if (moveGuarded(id, pos0 + d, SPEED, ACC, ms) != 0) status = 1;
    wait(300);
    if (!g_quiet) SAY("  back to start %d", pos0);
    if (moveGuarded(id, pos0, SPEED, ACC, ms) != 0) { SAY("id %d: could not get back to start, stopping", id); status = 2; break; }
    wait(300);
  }
  wr16(id, R_TQ_LIMIT, tl0);
  if (tq == 0) wr8(id, R_TORQUE, 0);
  if (!g_quiet) SAY("id %d: done, torque back to %d, cap back to %d%%", id, tq, tl0 / 10);
  return status;
}

// Absolute goals in order, guarded, no return to start. Returns 0 ok, 1 a move blocked or timed out, 3 read/limit failure.
// speed in steps/s; trip > 0 overrides the load trip (default 80 % of the cap).
static int runSeq(int id, const int *goals, uint8_t n, int tqPct, int speed = 150, int trip = 0) {
  char b1[10];
  int status = 0;
  g_runPeak = 0; g_runWorstErr = 0;
  g_tqLimit = tqPct * 10;
  const int SPEED = constrain(speed, 20, 300), ACC = 10;
  int pos0, mode, tq, mn, mx, tl0 = 1000;
  if (!rd16(id, R_POS, &pos0) || !rd8(id, R_MODE, &mode) || !rd8(id, R_TORQUE, &tq) ||
      !rd16(id, R_MIN_LIM, &mn) || !rd16(id, R_MAX_LIM, &mx)) { SAY("id %d: read failed", id); return 3; }
  if (mode != 0) { SAY("id %d: mode %d is not position mode", id, mode); return 3; }
  int lo = (mx > mn) ? mn : 0, hi = (mx > mn) ? mx : 4095;
  for (uint8_t i = 0; i < n; i++)
    if (goals[i] < lo || goals[i] > hi) { SAY("id %d: goal %d outside limits %d..%d", id, goals[i], lo, hi); return 3; }
  rd16(id, R_TQ_LIMIT, &tl0);
  wr16(id, R_TQ_LIMIT, g_tqLimit);
  wrGoal(id, pos0, SPEED, ACC);
  if (tq == 0) { wr8(id, R_TORQUE, 1); wait(50); }
  int prev = pos0;
  for (uint8_t i = 0; i < n; i++) {
    if (!g_quiet) SAY("  id %d -> %d (%s deg)", id, goals[i], deg(goals[i], b1));
    uint32_t ms = 1500 + 2000UL * abs(goals[i] - prev) / SPEED;
    g_tripOverride = trip;
    if (moveGuarded(id, goals[i], SPEED, ACC, ms) != 0) status = 1;
    g_tripOverride = 0;
    prev = goals[i];
    wait(300);
  }
  wr16(id, R_TQ_LIMIT, tl0);
  if (tq == 0) wr8(id, R_TORQUE, 0);
  if (!g_quiet) SAY("id %d: done, peak load %d", id, g_runPeak);
  return status;
}

// ---------------------------------------------------------------- endurance
// Test motion: ID 1 +/-SWING1 from where it starts, ID 2 full sweep between two absolute positions.
static const int SWING1 = 50;  // ID 1 swing, deg
static int g_hi2 = 1660;  // 145.9 deg, ~5 deg short of where ID 2 stalls (~151 deg)
static const int DOCK_SPEED = 50, DOCK_TRIP = 160;  // ID 2 into the port: these docked cleanly, no belt skip (11-24)
static int g_lo2 = 1080;  // 94.9 deg, ~4 deg short of the retract stop at ~91 deg (seen on every undock, 11-24)

// The C5 has no simple way to read its own supply, so that column is 0. Chip temperature is real (deg C).
static long boardVcc() { return 0; }
static int chipTemp() { return (int)lroundf(temperatureRead()); }

// Result record in flash (EEPROM emulation), so a run on buck power (no PC) can be read back with "report".
enum : uint8_t { ST_IDLE = 0, ST_ARMED = 1, ST_RUNNING = 2, ST_DONE = 3 };
enum : uint8_t { SC_TIME = 0, SC_PC, SC_STUCK, SC_TEMP, SC_VOLT, SC_READ, SC_BLOCKS, SC_RESET, SC_NONE = 255 };
static const char *const STOP_TXT[] = {"time up", "stopped from PC", "servo could not be read or got stuck",
                                       "servo temperature >= 60 C", "servo supply < 10.5 V", "status read failed",
                                       "3 blocked moves", "board reset during the run (brown-out?)"};
struct Rec {
  uint16_t magic;
  uint8_t state, stop;
  uint16_t armSec, cycles;
  uint32_t secs;
  uint16_t blocks, commErr;
  uint8_t maxT1, maxT2, minV1, minV2;
  uint16_t minVcc, maxVcc;
  int16_t maxDT;
  uint16_t boots;
};
static const uint16_t MAGIC = 0x5E12;
static Rec g_rec;
static void recLoad() {
  EEPROM.get(0, g_rec);
  if (g_rec.magic != MAGIC) { memset(&g_rec, 0, sizeof g_rec); g_rec.magic = MAGIC; g_rec.stop = SC_NONE; }
}
static void recSave() { EEPROM.put(0, g_rec); EEPROM.commit(); }  // flash-backed EEPROM emulation

static void ringFill(uint8_t r, uint8_t g, uint8_t b) {
  ring.setBrightness(24);
  ring.fill(ring.Color(r, g, b));
  ring.show();
}

// Runs the test motion over and over with the rainbow on, one summary line per cycle, for maxSec (0 = until
// stopped). Stops on 's' from the PC, or on: servo >= 60 C, servo supply < 10.5 V, a servo that cannot be read,
// or 3 blocked moves. persist: keep the stats in EEPROM (auto run on buck power).
static uint8_t cmdEndure(uint16_t maxSec, int n, bool persist) {
  g_quiet = true; g_anim = true; g_stopReq = false;
  ring.updateLength(n);
  ring.setBrightness(24);
  uint32_t t0 = millis(), err0 = g_commErr;
  uint16_t cycle = 0, blocks = 0;
  uint8_t stop = SC_PC;
  int home1 = -1;
  chipTemp();
  int temp0 = chipTemp();
  if (!rd16(1, R_POS, &home1)) {
    SAY("ENDURE STOP: cannot read id 1");
    g_quiet = g_anim = false;
    return SC_READ;
  }
  SAY("endure: id 1 +/-%d from %d, id 2 dock %d <-> %d at %d steps/s trip %d, cap 30%%, rainbow %d LEDs, %u s. 's' stops",
      SWING1, home1, g_hi2, g_lo2, DOCK_SPEED, DOCK_TRIP, n, maxSec);
  SAY("ENDURE cycle,t_s,id1,id1_peak,id1_err,id2,id2_peak,id2_err,t1_C,t2_C,v1_dV,v2_dV,board_mV,chip_dtemp_C,comm_err,blocks");
  while (true) {
    if (g_stopReq) { stop = SC_PC; break; }
    if (maxSec && (millis() - t0) / 1000 >= maxSec) { stop = SC_TIME; break; }
    cycle++;
    int m1[2] = {SWING1, -SWING1}, g2[2] = {g_hi2, g_lo2};
    int s1 = runMoves(1, m1, 2, 30, home1); int pk1 = g_runPeak, e1 = g_runWorstErr;
    int s2 = runSeq(2, g2, 2, 30, DOCK_SPEED, DOCK_TRIP); int pk2 = g_runPeak, e2 = g_runWorstErr;
    if (s1 == 1) blocks++;
    if (s2 == 1) blocks++;
    int t1 = -1, t2 = -1, v1 = -1, v2 = -1;
    rd8(1, R_TEMP, &t1); rd8(2, R_TEMP, &t2); rd8(1, R_VOLT, &v1); rd8(2, R_VOLT, &v2);
    long vcc = boardVcc();
    int dT = chipTemp() - temp0;
    uint32_t secs = (millis() - t0) / 1000;
    SAY("ENDURE %u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%d,%lu,%u", cycle, secs, s1, pk1, e1, s2, pk2, e2, t1, t2,
        v1, v2, vcc, dT, g_commErr - err0, blocks);
    if (persist) {
      g_rec.cycles = cycle; g_rec.secs = secs; g_rec.blocks = blocks; g_rec.commErr = g_commErr - err0;
      if (t1 > g_rec.maxT1) g_rec.maxT1 = t1;
      if (t2 > g_rec.maxT2) g_rec.maxT2 = t2;
      if (v1 >= 0 && v1 < g_rec.minV1) g_rec.minV1 = v1;
      if (v2 >= 0 && v2 < g_rec.minV2) g_rec.minV2 = v2;
      if (vcc < g_rec.minVcc) g_rec.minVcc = vcc;
      if (vcc > g_rec.maxVcc) g_rec.maxVcc = vcc;
      if (dT > g_rec.maxDT) g_rec.maxDT = dT;
      recSave();
    }
    if (s1 >= 2 || s2 >= 2) { stop = SC_STUCK; break; }
    if (t1 >= 60 || t2 >= 60) { stop = SC_TEMP; break; }
    if ((v1 >= 0 && v1 < 105) || (v2 >= 0 && v2 < 105)) { stop = SC_VOLT; break; }
    if (t1 < 0 || t2 < 0 || v1 < 0 || v2 < 0) { stop = SC_READ; break; }
    if (blocks >= 3) { stop = SC_BLOCKS; break; }
    wait(200);
  }
  g_quiet = false; g_anim = false;
  wr8(1, R_TORQUE, 0); wr8(2, R_TORQUE, 0);
  SAY("ENDURE STOP after %u cycles, %lu s: %s", cycle, (millis() - t0) / 1000, STOP_TXT[stop]);
  return stop;
}

// dock: send ID 2 out (+ direction) to g_hi2 with the guard on, then keep torque on at the 30 % cap so it
// stays seated. If the port stops it early, the guard holds it where it stopped instead of pushing.
// undock: pull ID 2 back to g_lo2, then torque off and the cap back to 100 %.
static void cmdDock(bool out, int speed, int trip) {
  char b1[10];
  const int SPEED = constrain(speed, 20, 300), ACC = 10;
  int pos0;
  if (!rd16(2, R_POS, &pos0)) { SAY("id 2: read failed"); return; }
  int goal = out ? g_hi2 : g_lo2;
  g_tqLimit = 300;
  g_runPeak = 0;
  wr16(2, R_TQ_LIMIT, 300);
  wrGoal(2, pos0, SPEED, ACC);
  wr8(2, R_TORQUE, 1);
  delay(50);
  g_tripOverride = trip;
  SAY("%s: id 2 from %d (%s deg) to %d, speed %d steps/s, trip load %d", out ? "dock" : "undock", pos0,
      deg(pos0, b1), goal, SPEED, trip > 0 ? trip : 240);
  int r = moveGuarded(2, goal, SPEED, ACC, 1500 + 2000UL * abs(goal - pos0) / SPEED);
  g_tripOverride = 0;
  int pos = -1, load = 0, raw;
  rd16(2, R_POS, &pos);
  if (rd16(2, R_LOAD, &raw)) load = loadSigned(raw);
  if (out) {
    SAY("dock: %s at %d (%s deg), load now %d, peak %d. holding, torque on, cap 30%%",
        r == 0 ? "reached" : "stopped early", pos, deg(pos, b1), load, g_runPeak);
  } else {
    wr16(2, R_TQ_LIMIT, 1000);
    wr8(2, R_TORQUE, 0);
    SAY("undock: %s at %d (%s deg), peak %d. torque off", r == 0 ? "reached" : "stopped early", pos, deg(pos, b1),
        g_runPeak);
  }
}

// push <deg> [speed] [cap%] [trip]: move ID 2 by deg from where it is, with the given speed (steps/s), torque cap
// and load trip, then hold there with torque on. For careful moves against the dock. Defaults 30 / 20 % / 100.
static void cmdPush(int id, int dDeg, int speed, int capPct, int trip, int stallMs = 400) {
  char b1[10];
  speed = constrain(speed, 20, 300);
  capPct = constrain(capPct, 5, 95);  // above 30 % only for freeing a stiff joint
  stallMs = constrain(stallMs, 200, 2000);
  int pos0;
  if (!rd16(id, R_POS, &pos0)) { SAY("id %d: read failed", id); return; }
  int goal = pos0 + (int)lroundf(dDeg * 4096.0f / 360.0f);
  if (goal < 0 || goal > 4095) { SAY("goal %d outside 0..4095", goal); return; }
  g_tqLimit = capPct * 10;
  g_tripOverride = trip;
  g_runPeak = 0;
  wr16(id, R_TQ_LIMIT, g_tqLimit);
  wrGoal(id, pos0, speed, 10);
  wr8(id, R_TORQUE, 1);
  delay(50);
  SAY("push: id %d from %d (%s deg) by %+d deg to %d, speed %d, cap %d%%, trip load %d", id, pos0, deg(pos0, b1), dDeg,
      goal, speed, capPct, trip);
  g_stallMs = stallMs;
  int r = moveGuarded(id, goal, speed, 10, 1500 + 2000UL * abs(goal - pos0) / speed + stallMs);
  g_tripOverride = 0;
  g_stallMs = 400;
  int pos = -1, load = 0, raw, cur = 0;
  rd16(id, R_POS, &pos);
  if (rd16(id, R_LOAD, &raw)) load = loadSigned(raw);
  rd16(id, R_CURRENT, &cur);
  int holdCap = min(capPct, 30);  // never hold at more than 30 %
  wr16(id, R_TQ_LIMIT, holdCap * 10);
  SAY("push: %s at %d (%s deg), moved %d steps, load now %d, current %d, peak %d. holding, torque on, cap %d%%",
      r == 0 ? "reached" : "stopped early", pos, deg(pos, b1), pos - pos0, load, cur, g_runPeak, holdCap);
}

static void cmdReport() {
  static const char *const ST[] = {"idle", "armed", "running", "done"};
  SAY("report: state %s, armed for %u s, boots %u", g_rec.state < 4 ? ST[g_rec.state] : "?", g_rec.armSec, g_rec.boots);
  if (g_rec.stop == SC_NONE) { SAY("  no auto run recorded"); return; }
  SAY("  stop: %s", g_rec.stop < 8 ? STOP_TXT[g_rec.stop] : "?");
  SAY("  %u cycles in %lu s, blocked moves %u, comm errors %u", g_rec.cycles, g_rec.secs, g_rec.blocks, g_rec.commErr);
  SAY("  servo max temp %u / %u C, servo min supply %u.%u / %u.%u V", g_rec.maxT1, g_rec.maxT2, g_rec.minV1 / 10,
      g_rec.minV1 % 10, g_rec.minV2 / 10, g_rec.minV2 % 10);
  SAY("  chip temp rise %d C over the run", g_rec.maxDT);
}

// Auto run on power-up after "arm <sec>": 10 s of blue blinks, then the endurance run, then the ring shows the
// result until power-off: dim green = clean, red = a fault. One shot: the arm is used up when the run starts.
static void autoRun() {
  g_rec.state = ST_RUNNING; g_rec.stop = SC_NONE;
  g_rec.cycles = 0; g_rec.secs = 0; g_rec.blocks = 0; g_rec.commErr = 0;
  g_rec.maxT1 = g_rec.maxT2 = 0; g_rec.minV1 = g_rec.minV2 = 255;
  g_rec.minVcc = 65535; g_rec.maxVcc = 0; g_rec.maxDT = -32768;
  recSave();
  ring.updateLength(16);
  for (uint8_t i = 0; i < 10; i++) { ringFill(0, 0, 255); delay(500); ringFill(0, 0, 0); delay(500); }
  uint8_t stop = cmdEndure(g_rec.armSec, 16, true);
  g_rec.state = ST_DONE; g_rec.stop = stop;
  recSave();
  bool clean = stop == SC_TIME && g_rec.blocks == 0 && g_rec.commErr == 0;
  if (clean) ringFill(0, 255, 0); else ringFill(255, 0, 0);
  SAY("auto run finished: %s", clean ? "clean (green)" : "fault (red)");
}

static void cmdRainbow(int n, int sec) {
  ring.updateLength(n);
  ring.setBrightness(24);  // ~10 %, keeps a 16-LED ring near 100 mA
  uint32_t t0 = millis();
  uint16_t hue = 0;
  while (millis() - t0 < (uint32_t)sec * 1000) {
    ring.rainbow(hue);
    ring.show();
    hue += 256;  // one full turn of the colour wheel every ~5 s
    delay(20);
  }
  SAY("rainbow: %d LEDs on D3 (GPIO%d), %d s, holding last frame", n, RING_PIN, sec);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  EEPROM.begin(64);
  if (!busOpen(BUS_PIN, 1000000)) Serial.println("bus open failed");
  ring.begin();
  ring.updateLength(16);
  ring.setBrightness(24);
  ring.clear(); ring.show();
  delay(100);
  recLoad();
  g_rec.boots++;
  if (g_rec.state == ST_RUNNING) {  // power was lost or the board reset in the middle of an auto run
    g_rec.state = ST_DONE; g_rec.stop = SC_RESET;
    recSave();
    ring.updateLength(16);
    ringFill(255, 0, 0);
    SAY("previous auto run was cut short by a reset or power loss. 'report' for its stats");
  } else if (g_rec.state == ST_ARMED) {
    recSave();
    SAY("armed: auto run of %u s starts in 10 s", g_rec.armSec);
    autoRun();
  } else recSave();
  SAY("c5 servo+led test ready (bus D2=GPIO%d, ring D3=GPIO%d). cmds: scan [maxid] | swing <id> <deg> [tq%%] | go <id> <deg> [tq%%] | rainbow [n] [sec] | off | raw <id> | line | endure [sec] | seq <id> <p1> [p2] | arm <sec> | disarm | report", BUS_PIN, RING_PIN);
  SAY("> ");
}

void loop() {
  static char line[40];
  static uint8_t len = 0;
  if (!Serial.available()) { ringTick(); return; }
  g_quietUntil = millis() + 300;  // the PC sends one wake byte, waits, then the command
  char c = Serial.read();
  if (c != '\n' && c != '\r') {
    if (len < sizeof line - 1 && c >= 0x20 && c < 0x7F) line[len++] = c;
    return;
  }
  if (len == 0) return;
  line[len] = 0;
  len = 0;

  if (!strcmp(line, "s")) return;  // a stop request that arrived while idle: nothing to stop
  g_stopReq = false;  // a new command; 's' during it stops it
  char *cmd = strtok(line, " ");
  char *a1 = strtok(nullptr, " "), *a2 = strtok(nullptr, " "), *a3 = strtok(nullptr, " ");
  int a = a1 ? atoi(a1) : -1, b = a2 ? atoi(a2) : -1, c3 = a3 ? atoi(a3) : -1;

  if (!strcmp(cmd, "scan")) cmdScan(a1 ? a : 20);
  else if (!strcmp(cmd, "swing") && a2) {
    if (b <= 0 || b > 180) SAY("deg must be 1..180");
    else { int m[2] = {b, -b}; runMoves(a, m, 2, a3 ? c3 : 30); }
  }
  else if (!strcmp(cmd, "go") && a2) {
    if (b == 0 || abs(b) > 180) SAY("deg must be nonzero, |deg| <= 180");
    else { int m[1] = {b}; runMoves(a, m, 1, a3 ? c3 : 30); }
  }
  else if (!strcmp(cmd, "endure")) cmdEndure(a1 ? a : 0, a2 ? b : 16, false);
  else if (!strcmp(cmd, "seq") && a2) { int g[2] = {b, c3}; runSeq(a, g, a3 ? 2 : 1, 30); }
  else if (!strcmp(cmd, "arm") && a1 && a > 0) {
    g_rec.state = ST_ARMED; g_rec.armSec = a; recSave();
    SAY("armed: the next power-up runs the test for %d s by itself (blue blinks for 10 s first)", a);
  }
  else if (!strcmp(cmd, "disarm")) { g_rec.state = ST_IDLE; recSave(); SAY("disarmed"); }
  else if (!strcmp(cmd, "report")) cmdReport();
  else if (!strcmp(cmd, "push") && a1) {
    char *a4 = strtok(nullptr, " ");
    char *a5 = strtok(nullptr, " ");
    cmdPush(2, a, a2 ? b : 30, a3 ? c3 : 20, a4 ? atoi(a4) : 100, a5 ? atoi(a5) : 400);
  }
  else if (!strcmp(cmd, "dock")) cmdDock(true, a1 ? a : DOCK_SPEED, a2 ? b : DOCK_TRIP);
  else if (!strcmp(cmd, "undock")) cmdDock(false, a1 ? a : DOCK_SPEED, a2 ? b : DOCK_TRIP);
  else if (!strcmp(cmd, "rainbow")) {
    if (a1) cmdRainbow(a, a2 ? b : 10);
    else { g_rainbow = true; SAY("rainbow on"); }
  }
  else if (!strcmp(cmd, "off")) { g_rainbow = false; ring.clear(); ring.show(); SAY("led: off"); }
  else if (!strcmp(cmd, "raw") && a1) {
    uint8_t dummy[1];
    int r = txrx((uint8_t)a, 0x01, nullptr, 0, dummy, 0);
    char hex[200];
    int k = 0;
    for (int i = 0; i < lastRawN && k < (int)sizeof hex - 4; i++) k += snprintf(hex + k, sizeof hex - k, " %02X", lastRaw[i]);
    hex[k] = 0;
    SAY("raw ping id %d: result %d, %d bytes (echo first):%s", a, r, lastRawN, hex);
  }
  else if (!strcmp(cmd, "line")) {
    // Idle level with the weak pull-down, then pull-up. A powered servo's pull-up wins over the pull-down.
    gpio_set_pull_mode((gpio_num_t)BUS_PIN, GPIO_PULLDOWN_ONLY); delay(20);
    int lo = gpio_get_level((gpio_num_t)BUS_PIN);
    gpio_set_pull_mode((gpio_num_t)BUS_PIN, GPIO_PULLUP_ONLY); delay(20);
    int hi = gpio_get_level((gpio_num_t)BUS_PIN);
    SAY("line D2 (GPIO%d): pull-down reads %d, pull-up reads %d -> %s", BUS_PIN, lo, hi,
        lo && hi ? "held high (servo powered)" : !lo && !hi ? "held low" : "floating (nothing on it)");
  }
  else if ((!strcmp(cmd, "ledpin") || !strcmp(cmd, "pinhigh")) && a1) {
    // Diagnostics for the ring. Refuses the servo bus pin and the C5 boot-strapping pins.
    bool bad = a == BUS_PIN || a == 2 || a == 3 || a == 26 || a == 27 || a == 28 || a < 0 || a > 28;
    if (bad) SAY("refused: GPIO%d is the servo bus, a strapping pin or out of range", a);
    else if (!strcmp(cmd, "ledpin")) {
      ring.clear(); ring.show();
      pinMode(RING_PIN, INPUT);
      RING_PIN = a;
      ring.setPin(a);
      ring.begin();
      g_rainbow = true;
      SAY("ring data now on GPIO%d, rainbow on", a);
    } else {
      g_rainbow = false;
      ring.clear(); ring.show();
      pinMode(a, OUTPUT);
      digitalWrite(a, HIGH);
      SAY("GPIO%d held at 3.3 V (rainbow off). Measure DIN to GND at the ring. 'rainbow' or 'ledpin' to undo", a);
    }
  }
  else if (!strcmp(cmd, "move") && a2) {
    // move <id> <deg> [speed] [cap%] [trip] [stall_ms]: guarded move of any servo, then hold at <= 30 %.
    char *a4 = strtok(nullptr, " "), *a5 = strtok(nullptr, " "), *a6 = strtok(nullptr, " ");
    cmdPush(a, b, a3 ? c3 : 50, a4 ? atoi(a4) : 30, a5 ? atoi(a5) : 240, a6 ? atoi(a6) : 400);
  }
  else if (!strcmp(cmd, "stat") && a1) {
    // One line for the UI: STAT id pos load current volt_dV temp_C torque moving chip_C
    int pos = -1, raw = 0, cur = -1, v = -1, t = -1, tq = -1, mv = -1;
    bool ok = rd16(a, R_POS, &pos);
    int load = rd16(a, R_LOAD, &raw) ? loadSigned(raw) : 0;
    rd16(a, R_CURRENT, &cur); rd8(a, R_VOLT, &v); rd8(a, R_TEMP, &t); rd8(a, R_TORQUE, &tq); rd8(a, 66, &mv);
    SAY("STAT %d %d %d %d %d %d %d %d %.1f %d", a, ok ? pos : -1, load, cur, v, t, tq, mv, temperatureRead(), ok ? 1 : 0);
  }
  else if (!strcmp(cmd, "relax") && a1) {
    // Torque off and the torque cap back to 100 %. The joint goes limp where it is.
    int pos = -1, load = 0, raw;
    rd16(a, R_POS, &pos);
    if (rd16(a, R_LOAD, &raw)) load = loadSigned(raw);
    bool ok = wr8(a, R_TORQUE, 0);
    wr16(a, R_TQ_LIMIT, 1000);
    SAY("relax id %d: %s at %d, load before %d", a, ok ? "torque off" : "write failed", pos, load);
  }
  else if (!strcmp(cmd, "temp")) SAY("chip temperature %.1f C, up %lu s", temperatureRead(), millis() / 1000);
  else if (!strcmp(cmd, "drive") && a1) {
    ppTx = a != 0;
    SAY("drive: %s while sending", ppTx ? "push-pull" : "open-drain");
  }
  else SAY("? scan [maxid] | swing <id> <deg> [tq%%] | go <id> <deg> [tq%%] | rainbow [n] [sec] | off | raw <id> | line | endure [sec] | seq <id> <p1> [p2] | arm <sec> | disarm | report");
  SAY("> ");
}
