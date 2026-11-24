// Bench test for a Seeed XIAO ESP32-C5: FEETECH STS3215 12 V bus servos and a WS2812B ring.
// The servo data wire goes straight to one GPIO. No adapter board.
// Nothing moves on boot. Commands over USB serial (115200):
//   scan <gpio> [baud] [maxid]  ping servo IDs 0..maxid (default 30) on that pin. Read only. Default 1000000.
//   nudge <deg> [id] [tq%]      +deg, back, -deg, back, slowly. Max 45. Torque capped (default 30 %);
//                               stops and backs off if the joint is blocked.
//   go <id> <deg> [tq%]         one guarded move by deg (negative = other way), then back. |deg| <= 90.
//   led <gpio> [n]              ring test: red, green, blue, chase, low brightness, left dim blue. n = 16.
//   off                         LEDs off.
//   raw <id>                    ping one ID (254 = broadcast) and hex-dump the wire.
//   line <gpio>                 idle level with pull-down, then pull-up (is anything driving the pin?).
//   drive <0|1>                 1 (default): push-pull while sending. 0: open drain only.
// XIAO ESP32-C5 pins: D0 = GPIO1, D1 = GPIO0, D2 = GPIO25. (XIAO ESP32-S3: D0 = GPIO1, D1 = GPIO2, D2 = GPIO3.)
// Bench wiring on 2026-11-24: servo data on D2 (GPIO25), ring DIN on D1 (GPIO0).
//
// The servo bus is one half-duplex wire, so TX and RX share the pad. The pad is open drain with
// pull-up while listening, and push-pull while sending. Open drain alone did not work: the servo
// never answered, likely because its weak pull-up gives slow rising edges at 1 Mbit/s. The pad goes
// back to open drain as soon as the last stop bit is out, before the servo can reply.
// Our own bytes echo back and are skipped.
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_reg.h>
#include <esp_rom_gpio.h>
#include <hal/uart_ll.h>

static const uart_port_t BUS = UART_NUM_1;
static int busPin = -1;
static int busBaud = 1000000;
// true: drive the pad push-pull while sending (sharp rising edges), switch back to open drain the moment
// the last stop bit is out, before the servo can answer. false: open drain the whole time.
static bool ppTx = true;

static uint8_t found[32];
static int nFound = 0;

static Adafruit_NeoPixel *ring = nullptr;
static int ringPin = -1;

// STS3215 registers (little endian)
enum : uint8_t {
  R_MIN_LIM = 9, R_MAX_LIM = 11, R_MODE = 33, R_TORQUE = 40, R_ACC = 41, R_GOAL = 42,
  R_POS = 56, R_LOAD = 60, R_VOLT = 62, R_TEMP = 63, R_MOVING = 66, R_CURRENT = 69
};

// ---------------------------------------------------------------- bus
static void busClose() {
  if (uart_is_driver_installed(BUS)) uart_driver_delete(BUS);
  if (busPin >= 0) gpio_reset_pin((gpio_num_t)busPin);
  busPin = -1;
}

static void ledRelease() {
  if (ring) { ring->clear(); ring->show(); delete ring; ring = nullptr; }
  if (ringPin >= 0) gpio_reset_pin((gpio_num_t)ringPin);
  ringPin = -1;
}

static bool busOpen(int pin, int baud) {
  busClose();
  if (pin == ringPin) ledRelease();
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
  busPin = pin;
  busBaud = baud;
  uint32_t padReg = REG_READ(GPIO_PIN0_REG + 4 * pin);
  uint32_t outSel = REG_READ(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * pin) & 0x1FF;
  bool oe = (pin < 32) ? (REG_READ(GPIO_ENABLE_REG) >> pin) & 1 : 0;
  Serial.printf("bus: GPIO%d @ %d baud, open-drain=%lu out_sel=%lu (U1TXD=%d) oe=%d\n", pin, baud,
                (unsigned long)((padReg >> 2) & 1), (unsigned long)outSel, U1TXD_OUT_IDX, oe);
  return true;
}

static void busSend(const uint8_t *pkt, int k) {
  uint32_t reg = GPIO_PIN0_REG + 4 * busPin;
  if (ppTx) REG_CLR_BIT(reg, BIT(GPIO_PIN0_PAD_DRIVER_S));
  uart_write_bytes(BUS, pkt, k);
  if (ppTx) {
    uint32_t t0 = micros();
    while (!uart_ll_is_tx_idle(UART_LL_GET_HW(BUS)) && micros() - t0 < 20000) {}
    REG_SET_BIT(reg, BIT(GPIO_PIN0_PAD_DRIVER_S));
  }
  uart_wait_tx_done(BUS, pdMS_TO_TICKS(10));
}

// Sends one packet, returns number of response params (>= 0), or < 0 on error.
// echoOk reports whether our own bytes came back intact (a wiring / rise-time check).
static int txrx(uint8_t id, uint8_t ins, const uint8_t *p, int n, uint8_t *out, int outLen,
                uint8_t *errOut = nullptr, bool *echoOk = nullptr, int timeoutMs = 6) {
  uint8_t pkt[48];
  int k = 0;
  pkt[k++] = 0xFF; pkt[k++] = 0xFF; pkt[k++] = id; pkt[k++] = n + 2; pkt[k++] = ins;
  uint8_t sum = id + (n + 2) + ins;
  for (int i = 0; i < n; i++) { pkt[k++] = p[i]; sum += p[i]; }
  pkt[k++] = ~sum;

  uart_flush_input(BUS);
  busSend(pkt, k);

  uint8_t buf[128];
  int want = k + 6 + outLen, got = 0;
  uint32_t t0 = millis();
  while (got < want && millis() - t0 < (uint32_t)timeoutMs) {
    int r = uart_read_bytes(BUS, buf + got, want - got, pdMS_TO_TICKS(1));
    if (r > 0) got += r;
  }
  int start = 0;
  bool echo = got >= k && memcmp(buf, pkt, k) == 0;
  if (echoOk) *echoOk = echo;
  if (echo) start = k;
  if (id == 0xFE) return 0;  // broadcast, no reply
  for (int i = start; i + 5 < got; i++) {
    if (buf[i] != 0xFF || buf[i + 1] != 0xFF || buf[i + 2] != id) continue;
    int len = buf[i + 3];
    if (i + 3 + len >= got) return -3;  // short
    uint8_t s = 0;
    for (int j = 2; j < 3 + len; j++) s += buf[i + j];
    if ((uint8_t)~s != buf[i + 3 + len]) return -4;  // checksum
    if (errOut) *errOut = buf[i + 4];
    int np = len - 2;
    for (int j = 0; j < np && j < outLen; j++) out[j] = buf[i + 5 + j];
    return np;
  }
  return got > start ? -2 : -1;  // -1 silence, -2 garbage
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
static bool wr(uint8_t id, uint8_t addr, const uint8_t *d, int n) {
  uint8_t p[16];
  p[0] = addr;
  memcpy(p + 1, d, n);
  uint8_t dummy[1];
  return txrx(id, 0x03, p, n + 1, dummy, 0) >= 0;
}
static bool wrGoal(uint8_t id, int goal, int speed, int acc) {
  // 41 acc, 42-43 goal, 44-45 time, 46-47 speed
  uint8_t d[7] = {(uint8_t)acc, (uint8_t)(goal & 0xFF), (uint8_t)(goal >> 8), 0, 0,
                  (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)};
  return wr(id, R_ACC, d, 7);
}

static void cmdScan(int pin, int baud, int maxId = 30) {
  if (!busOpen(pin, baud)) { Serial.println("bus open failed"); return; }
  nFound = 0;
  bool anyEcho = false;
  for (int id = 0; id <= maxId && nFound < 32; id++) {
    uint8_t dummy[1], err = 0;
    bool echo = false;
    int r = txrx(id, 0x01, nullptr, 0, dummy, 0, &err, &echo, 4);
    anyEcho |= echo;
    if (r == -2 || r == -3 || r == -4) Serial.printf("  id %d: bad reply (%d)\n", id, r);
    if (r < 0) continue;
    found[nFound++] = id;
    int pos = -1, mode = -1, tq = -1, v = -1, t = -1, mn = -1, mx = -1;
    rd16(id, R_POS, &pos); rd8(id, R_MODE, &mode); rd8(id, R_TORQUE, &tq);
    rd8(id, R_VOLT, &v); rd8(id, R_TEMP, &t); rd16(id, R_MIN_LIM, &mn); rd16(id, R_MAX_LIM, &mx);
    Serial.printf("  FOUND id %d: pos %d (%.1f deg) mode %d torque %d  %.1f V  %d C  limits %d..%d  err 0x%02X\n",
                  id, pos, pos * 360.0 / 4096, mode, tq, v / 10.0, t, mn, mx, err);
  }
  Serial.printf("scan done: %d servo(s) on GPIO%d. echo %s\n", nFound, pin,
                anyEcho ? "ok" : "MISSING (TX not reaching the pad, or the line is held low)");
}

// Torque limit register (RAM, 0..1000 = 0..100 % of max torque). Resets to the EEPROM value on power-up.
static const uint8_t R_TQ_LIMIT = 48;
static int g_tqLimit = 300;  // set per nudge

static int loadSigned(int raw) { return (raw & 0x400) ? -(raw & 0x3FF) : (raw & 0x3FF); }

// 0 = reached, 1 = blocked (load too high or stopped short), 2 = timeout / read error.
// Prints a short trace so you can see what the joint is doing.
static int moveGuarded(uint8_t id, int goal, int speed, int acc, int ms) {
  wrGoal(id, goal, speed, acc);
  const int loadTrip = g_tqLimit * 8 / 10;  // pushing at 80 % of the cap = something is in the way
  uint32_t t0 = millis(), lastProg = millis(), lastPrint = 0;
  int pos = -1, lastPos = -1, load = 0, cur = 0, peak = 0, raw;
  while (millis() - t0 < (uint32_t)ms) {
    if (!rd16(id, R_POS, &pos)) { delay(10); continue; }
    if (rd16(id, R_LOAD, &raw)) load = loadSigned(raw);
    rd16(id, R_CURRENT, &cur);
    if (abs(load) > abs(peak)) peak = load;
    if (millis() - lastPrint >= 150) {
      lastPrint = millis();
      Serial.printf("      t=%4lums pos %d (%+.1f deg to go) load %d cur %d\n", (unsigned long)(millis() - t0),
                    pos, (goal - pos) * 360.0 / 4096, load, cur);
    }
    if (abs(pos - goal) <= 8) {
      Serial.printf("    reached %d (%+.1f deg off), peak load %d\n", pos, (pos - goal) * 360.0 / 4096, peak);
      return 0;
    }
    if (lastPos < 0 || abs(pos - lastPos) >= 3) { lastPos = pos; lastProg = millis(); }
    bool tooHard = abs(load) >= loadTrip;
    bool stuck = millis() - t0 > 300 && millis() - lastProg > 400;
    if (tooHard || stuck) {
      wrGoal(id, pos, speed, acc);  // stop pushing: hold where it is
      Serial.printf("    BLOCKED at %d (%.1f deg short): %s, load %d (trip %d)\n", pos,
                    fabs(goal - pos) * 360.0 / 4096, tooHard ? "load over trip" : "stopped moving", load, loadTrip);
      return 1;
    }
    delay(15);
  }
  wrGoal(id, pos >= 0 ? pos : goal, speed, acc);
  Serial.printf("    TIMEOUT at %d\n", pos);
  return 2;
}

// nudge <deg> [id] [torque %]: +deg, back, -deg, back. Torque capped, stops and backs off if blocked.
static void cmdNudge(float deg, int onlyId, int tqPct) {
  if (busPin < 0 || nFound == 0) { Serial.println("scan first"); return; }
  if (deg <= 0 || deg > 45) { Serial.println("deg must be in (0, 45]"); return; }
  if (tqPct < 5 || tqPct > 100) { Serial.println("torque % must be 5..100"); return; }
  g_tqLimit = tqPct * 10;
  int d = (int)lroundf(deg * 4096.0f / 360.0f);
  const int SPEED = 150, ACC = 10;  // steps/s (~13 deg/s), gentle accel
  const int MS = 1500 + 2000 * d / SPEED;
  for (int i = 0; i < nFound; i++) {
    uint8_t id = found[i];
    if (onlyId >= 0 && id != onlyId) continue;
    int pos0, mode, tq, mn, mx;
    if (!rd16(id, R_POS, &pos0) || !rd8(id, R_MODE, &mode) || !rd8(id, R_TORQUE, &tq) ||
        !rd16(id, R_MIN_LIM, &mn) || !rd16(id, R_MAX_LIM, &mx)) {
      Serial.printf("id %d: read failed, skipped\n", id); continue;
    }
    if (mode != 0) { Serial.printf("id %d: mode %d is not position mode, skipped\n", id, mode); continue; }
    int lo = (mx > mn) ? mn : 0, hi = (mx > mn) ? mx : 4095;
    if (pos0 - d < lo || pos0 + d > hi) {
      Serial.printf("id %d: pos %d too close to limits %d..%d, skipped\n", id, pos0, lo, hi); continue;
    }
    int tl0 = 1000;
    rd16(id, R_TQ_LIMIT, &tl0);
    Serial.printf("id %d: start %d (%.1f deg), torque was %d, +/-%.1f deg (%d steps), torque cap %d%% (was %d%%)\n",
                  id, pos0, pos0 * 360.0 / 4096, tq, deg, d, tqPct, tl0 / 10);
    uint8_t tl[2] = {(uint8_t)(g_tqLimit & 0xFF), (uint8_t)(g_tqLimit >> 8)};
    wr(id, R_TQ_LIMIT, tl, 2);
    wrGoal(id, pos0, SPEED, ACC);  // hold where it is before enabling torque, so it does not jump
    if (tq == 0) { uint8_t on = 1; wr(id, R_TORQUE, &on, 1); delay(50); }
    const char *names[2] = {"+", "-"};
    for (int dir = 0; dir < 2; dir++) {
      int goal = dir == 0 ? pos0 + d : pos0 - d;
      Serial.printf("  %s%.1f deg -> %d\n", names[dir], deg, goal);
      int r = moveGuarded(id, goal, SPEED, ACC, MS);
      delay(300);
      Serial.printf("  back to start %d\n", pos0);
      if (moveGuarded(id, pos0, SPEED, ACC, MS) != 0) {
        Serial.printf("id %d: could not get back to start, stopping here\n", id);
        break;
      }
      delay(300);
      (void)r;
    }
    uint8_t tlb[2] = {(uint8_t)(tl0 & 0xFF), (uint8_t)(tl0 >> 8)};
    wr(id, R_TQ_LIMIT, tlb, 2);
    if (tq == 0) { uint8_t off = 0; wr(id, R_TORQUE, &off, 1); }
    Serial.printf("id %d: done, torque restored to %d, torque cap back to %d%%\n", id, tq, tl0 / 10);
  }
}

// go <id> <deg> [tq%]: one guarded move of <deg> (sign = direction) from where it is, then back to start.
static void cmdGo(int id, float deg, int tqPct) {
  if (busPin < 0) { Serial.println("scan first"); return; }
  if (deg == 0 || fabsf(deg) > 90) { Serial.println("deg must be nonzero, |deg| <= 90"); return; }
  if (tqPct < 5 || tqPct > 100) { Serial.println("torque % must be 5..100"); return; }
  g_tqLimit = tqPct * 10;
  int d = (int)lroundf(deg * 4096.0f / 360.0f);
  const int SPEED = 150, ACC = 10;
  const int MS = 1500 + 2000 * abs(d) / SPEED;
  int pos0, mode, tq, mn, mx, tl0 = 1000;
  if (!rd16(id, R_POS, &pos0) || !rd8(id, R_MODE, &mode) || !rd8(id, R_TORQUE, &tq) ||
      !rd16(id, R_MIN_LIM, &mn) || !rd16(id, R_MAX_LIM, &mx)) { Serial.printf("id %d: read failed\n", id); return; }
  if (mode != 0) { Serial.printf("id %d: mode %d is not position mode\n", id, mode); return; }
  int lo = (mx > mn) ? mn : 0, hi = (mx > mn) ? mx : 4095;
  int goal = pos0 + d;
  if (goal < lo || goal > hi) { Serial.printf("id %d: goal %d outside limits %d..%d\n", id, goal, lo, hi); return; }
  rd16(id, R_TQ_LIMIT, &tl0);
  Serial.printf("id %d: start %d (%.1f deg), %+.1f deg -> %d, torque cap %d%%\n", id, pos0, pos0 * 360.0 / 4096,
                deg, goal, tqPct);
  uint8_t tl[2] = {(uint8_t)(g_tqLimit & 0xFF), (uint8_t)(g_tqLimit >> 8)};
  wr(id, R_TQ_LIMIT, tl, 2);
  wrGoal(id, pos0, SPEED, ACC);
  if (tq == 0) { uint8_t on = 1; wr(id, R_TORQUE, &on, 1); delay(50); }
  moveGuarded(id, goal, SPEED, ACC, MS);
  delay(500);
  Serial.printf("  back to start %d\n", pos0);
  moveGuarded(id, pos0, SPEED, ACC, MS);
  uint8_t tlb[2] = {(uint8_t)(tl0 & 0xFF), (uint8_t)(tl0 >> 8)};
  wr(id, R_TQ_LIMIT, tlb, 2);
  if (tq == 0) { uint8_t off = 0; wr(id, R_TORQUE, &off, 1); }
  Serial.printf("id %d: done, torque restored to %d, torque cap back to %d%%\n", id, tq, tl0 / 10);
}

// ---------------------------------------------------------------- LEDs
static void cmdLed(int pin, int n) {
  ledRelease();
  if (pin == busPin) busClose();
  ring = new Adafruit_NeoPixel(n, pin, NEO_GRB + NEO_KHZ800);
  ring->begin();
  ring->setBrightness(24);  // ~10 %: 16 LEDs full white would pull ~1 A at 5 V
  ringPin = pin;
  const uint32_t cols[3] = {ring->Color(255, 0, 0), ring->Color(0, 255, 0), ring->Color(0, 0, 255)};
  const char *names[3] = {"red", "green", "blue"};
  for (int c = 0; c < 3; c++) {
    Serial.printf("led: all %s\n", names[c]);
    ring->fill(cols[c]); ring->show(); delay(700);
  }
  Serial.println("led: chase");
  for (int k = 0; k < 2 * n; k++) {
    ring->clear(); ring->setPixelColor(k % n, ring->Color(255, 255, 255)); ring->show(); delay(60);
  }
  ring->fill(ring->Color(0, 40, 255)); ring->show();
  Serial.printf("led: GPIO%d, %d px, left on dim blue\n", pin, n);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nservo+led test ready. cmds: scan <gpio> [baud] [maxid] | nudge <deg> [id] [tq%] | go <id> <deg> [tq%] | led <gpio> [n] | off | raw <id> | line <gpio> | drive <0|1>");
}

void loop() {
  if (!Serial.available()) { delay(5); return; }
  String line = Serial.readStringUntil('\n');
  line.trim();
  char cmd[16] = {0};
  float a = -1, b = -1, c3 = -1;
  int n = sscanf(line.c_str(), "%15s %f %f %f", cmd, &a, &b, &c3);
  if (n < 1) return;
  if (!strcmp(cmd, "scan") && n >= 2) cmdScan((int)a, n >= 3 ? (int)b : 1000000, n >= 4 ? (int)c3 : 30);
  else if (!strcmp(cmd, "nudge") && n >= 2) cmdNudge(a, n >= 3 ? (int)b : -1, n >= 4 ? (int)c3 : 30);
  else if (!strcmp(cmd, "go") && n >= 3) cmdGo((int)a, b, n >= 4 ? (int)c3 : 30);
  else if (!strcmp(cmd, "led") && n >= 2) cmdLed((int)a, n >= 3 ? (int)b : 16);
  else if (!strcmp(cmd, "off")) { ledRelease(); Serial.println("led: off"); }
  else if (!strcmp(cmd, "raw") && n >= 2) {
    // Ping one ID (254 = broadcast) and hex-dump everything on the wire for 20 ms, echo included.
    if (busPin < 0) { Serial.println("scan first"); }
    else {
      uint8_t id = (uint8_t)a, len = 2, ins = 1, chk = ~(uint8_t)(id + len + ins);
      uint8_t pkt[6] = {0xFF, 0xFF, id, len, ins, chk};
      uart_flush_input(BUS);
      busSend(pkt, 6);
      uint8_t buf[256]; int got = 0; uint32_t t0 = millis();
      while (millis() - t0 < 20 && got < 256) { int r = uart_read_bytes(BUS, buf + got, 256 - got, pdMS_TO_TICKS(2)); if (r > 0) got += r; }
      Serial.printf("raw ping id %d: %d bytes:", id, got);
      for (int i = 0; i < got; i++) Serial.printf(" %02X", buf[i]);
      Serial.println();
    }
  }
  else if (!strcmp(cmd, "drive") && n >= 2) {
    ppTx = (int)a != 0;
    Serial.printf("drive: %s while sending\n", ppTx ? "push-pull" : "open-drain");
  }
  else if (!strcmp(cmd, "line") && n >= 2) {
    // Idle level with the weak internal pull-down, then pull-up. A powered servo's pull-up wins over the pull-down.
    int pin = (int)a;
    if (pin == busPin) busClose();
    if (pin == ringPin) ledRelease();
    gpio_reset_pin((gpio_num_t)pin);
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY); delay(20);
    int lo = gpio_get_level((gpio_num_t)pin);
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLUP_ONLY); delay(20);
    int hi = gpio_get_level((gpio_num_t)pin);
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_FLOATING);
    Serial.printf("line GPIO%d: with pull-down reads %d, with pull-up reads %d -> %s\n", pin, lo, hi,
                  lo && hi ? "something holds it high (servo powered?)" :
                  !lo && !hi ? "something holds it low" : "floating (nothing driving it)");
  }
  else Serial.println("? scan <gpio> [baud] [maxid] | nudge <deg> [id] [tq%] | go <id> <deg> [tq%] | led <gpio> [n] | off | raw <id> | line <gpio> | drive <0|1>");
  Serial.println("> ");
}
