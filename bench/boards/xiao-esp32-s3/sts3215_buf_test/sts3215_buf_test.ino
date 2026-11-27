// Bench test for a Seeed XIAO ESP32-S3 on the EE2 servo bus buffer board (74LS125, drawing TC-EE2-BUF-001):
// FEETECH STS3215 bus servos at 1 Mbit/s through UART1 with separate TX, RX and DIR pins.
//
// Wiring (the XIAO in the board's socket): TX D7, RX D6, DIR D8 (low = send). The ESP hears its own bytes as an
// echo; they are checked and skipped. Power the board from USB OR the buck, never both.
//
// Commands over USB serial (newline terminated). Nothing moves on boot.
//   bustest                      loop check with no servo needed: does RX follow TX while DIR is low?
//   low [sec]                    hold DATA low (default 20 s) to measure its level at U1 pin 4.
//   bauds                        broadcast ping at every servo baud rate. Read only.
//   scan [maxid]                 ping IDs 0..maxid (default 20). Read only.
//   stat <id>                    position, load, volts, temperature, torque. Read only.
//   raw <id>                     one ping, bytes received printed in hex.
//   swing <id> <deg> [cap%] [speed]   +deg, back, -deg, back, then torque off. Defaults 30 %, 150 steps/s.
//   relax <id>                   torque off.
// Every move caps torque (register 48), holds the current position before enabling torque, and stops and holds if
// the load reaches 80 % of the cap or the joint stops for 0.4 s. Sending 's' stops a move part-way.
#include <Arduino.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <hal/uart_ll.h>

static const int PIN_TX = D7, PIN_RX = D6, PIN_DIR = D8;
static const uart_port_t BUS = UART_NUM_1;

enum : uint8_t { R_MODE = 33, R_TORQUE = 40, R_ACC = 41, R_TQ_LIMIT = 48, R_POS = 56 };

static void say(const char *fmt, ...) {
  char b[220];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  Serial.println(b);
}

// ---------------------------------------------------------------- servo bus
static bool busOpen(int baud = 1000000) {
  if (uart_is_driver_installed(BUS)) uart_driver_delete(BUS);
  gpio_reset_pin((gpio_num_t)PIN_TX);  // GPIO43/44 boot as UART0 pins on the S3
  gpio_reset_pin((gpio_num_t)PIN_RX);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);  // driver off
  uart_config_t c = {};
  c.baud_rate = baud;
  c.data_bits = UART_DATA_8_BITS;
  c.parity = UART_PARITY_DISABLE;
  c.stop_bits = UART_STOP_BITS_1;
  c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  c.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(BUS, 512, 0, 0, NULL, 0) != ESP_OK) return false;
  if (uart_param_config(BUS, &c) != ESP_OK) return false;
  return uart_set_pin(BUS, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK;
}

static void busSend(const uint8_t *pkt, int k) {
  digitalWrite(PIN_DIR, LOW);
  uart_write_bytes(BUS, pkt, k);
  uint32_t t0 = micros();
  while (!uart_ll_is_tx_idle(UART_LL_GET_HW(BUS)) && micros() - t0 < 20000) {}
  digitalWrite(PIN_DIR, HIGH);  // driver off before the servo answers
}

static uint8_t lastRaw[128];
static int lastRawN = 0;

// Sends one packet. Returns number of reply params (>= 0), or < 0: -1 silence, -2 garbage, -3 short, -4 checksum,
// -5 no echo of our own bytes, -6 echo corrupt.
static int txrx(uint8_t id, uint8_t ins, const uint8_t *p, int n, uint8_t *out, int outLen) {
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
  if (got < k) return -5;
  if (memcmp(lastRaw, pkt, k) != 0) return -6;
  for (int i = k; i + 5 < got; i++) {
    if (lastRaw[i] != 0xFF || lastRaw[i + 1] != 0xFF || lastRaw[i + 2] != id) continue;
    int len = lastRaw[i + 3];
    if (i + 3 + len >= got) return -3;
    uint8_t s = 0;
    for (int j = 2; j < 3 + len; j++) s += lastRaw[i + j];
    if ((uint8_t)~s != lastRaw[i + 3 + len]) return -4;
    int np = len - 2;
    for (int j = 0; j < np && j < outLen; j++) out[j] = lastRaw[i + 5 + j];
    return np;
  }
  return got > k ? -2 : -1;
}

static bool rd(uint8_t id, uint8_t addr, uint8_t len, uint8_t *out) {
  uint8_t p[2] = {addr, len};
  for (int t = 0; t < 3; t++)
    if (txrx(id, 0x02, p, 2, out, len) == len) return true;
  return false;
}
static bool wr(uint8_t id, uint8_t addr, const uint8_t *d, uint8_t n) {
  uint8_t p[16], dummy[1];
  p[0] = addr;
  memcpy(p + 1, d, n);
  for (int t = 0; t < 3; t++)
    if (txrx(id, 0x03, p, n + 1, dummy, 0) >= 0) return true;
  return false;
}
static bool wr8(uint8_t id, uint8_t addr, uint8_t v) { return wr(id, addr, &v, 1); }
static bool wr16(uint8_t id, uint8_t addr, int v) {
  uint8_t d[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
  return wr(id, addr, d, 2);
}
static bool wrGoal(uint8_t id, int goal, int speed, int acc) {  // 41 acc, 42-43 goal, 44-45 time, 46-47 speed
  uint8_t d[7] = {(uint8_t)acc, (uint8_t)(goal & 0xFF), (uint8_t)(goal >> 8), 0, 0,
                  (uint8_t)(speed & 0xFF), (uint8_t)(speed >> 8)};
  return wr(id, R_ACC, d, 7);
}
static int loadSigned(int raw) { return (raw & 0x400) ? -(raw & 0x3FF) : (raw & 0x3FF); }

// pos, load (0.1 %), volt (0.1 V), temp from one read of 56..63
static bool readState(uint8_t id, int *pos, int *load, int *volt, int *temp) {
  uint8_t b[8];
  if (!rd(id, R_POS, 8, b)) return false;
  *pos = b[0] | (b[1] << 8);
  *load = loadSigned(b[4] | (b[5] << 8));
  *volt = b[6];
  *temp = b[7];
  return true;
}

// ---------------------------------------------------------------- commands
static void cmdBustest() {
  uart_driver_delete(BUS);
  pinMode(PIN_TX, OUTPUT);
  pinMode(PIN_RX, INPUT);
  int r[4];
  for (int i = 0; i < 4; i++) {
    digitalWrite(PIN_DIR, i < 2 ? LOW : HIGH);
    digitalWrite(PIN_TX, i & 1);
    delay(5);
    r[i] = digitalRead(PIN_RX);
  }
  digitalWrite(PIN_DIR, HIGH);
  bool ok = r[0] == 0 && r[1] == 1 && r[2] == 1 && r[3] == 1;
  say("bustest: driver on: tx0->rx%d tx1->rx%d | driver off: tx0->rx%d tx1->rx%d -> %s", r[0], r[1], r[2], r[3],
      ok ? "OK, the buffer loop works" : r[0] == 1 ? "RX never goes low" : "wrong");
  say("bus reopen: %s", busOpen() ? "ok" : "FAILED");
}

// Holds DATA low (driver on, TX low) so its level can be measured with a multimeter at U1 pin 4.
static void cmdLow(int sec) {
  uart_driver_delete(BUS);
  pinMode(PIN_TX, OUTPUT);
  digitalWrite(PIN_TX, LOW);
  digitalWrite(PIN_DIR, LOW);
  say("DATA held low for %d s: measure U1 pin 4 to GND now", sec);
  delay(sec * 1000);
  digitalWrite(PIN_DIR, HIGH);
  say("released. bus reopen: %s", busOpen() ? "ok" : "FAILED");
}

static void cmdStat(uint8_t id) {
  int pos, load, volt, temp;
  uint8_t tq[1] = {0}, mode[1] = {0};
  if (!readState(id, &pos, &load, &volt, &temp)) { say("id %d: no reply", id); return; }
  rd(id, R_TORQUE, 1, tq);
  rd(id, R_MODE, 1, mode);
  say("id %d: pos %d (%.1f deg) load %+.1f%% %.1f V %d C torque %d mode %d", id, pos, pos * 360.0 / 4096,
      load / 10.0, volt / 10.0, temp, tq[0], mode[0]);
}

static void cmdScan(int maxId) {
  int found = 0;
  for (int id = 0; id <= maxId; id++) {
    uint8_t dummy[1];
    if (txrx(id, 0x01, nullptr, 0, dummy, 0) < 0 && txrx(id, 0x01, nullptr, 0, dummy, 0) < 0) continue;
    found++;
    say("FOUND:");
    cmdStat(id);
  }
  say("scan done: %d servo(s)", found);
}

static void cmdRaw(uint8_t id) {
  uint8_t dummy[1];
  int r = txrx(id, 0x01, nullptr, 0, dummy, 0);
  char b[3 * 40 + 1] = "";
  for (int i = 0; i < lastRawN && i < 40; i++) sprintf(b + 3 * i, "%02X ", lastRaw[i]);
  say("raw ping id %d: result %d, %d bytes (echo first): %s", id, r, lastRawN, b);
}

// Broadcast ping at every baud rate the servo can be set to: any byte after our echo means something answered.
static void cmdBauds() {
  static const int B[] = {1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400};
  for (int baud : B) {
    busOpen(baud);
    delay(20);
    uint8_t dummy[1];
    txrx(0xFE, 0x01, nullptr, 0, dummy, 0);
    delay(30);  // slow rates: the reply is still arriving after txrx's 6 ms window
    int more = uart_read_bytes(BUS, lastRaw + lastRawN, sizeof lastRaw - lastRawN, pdMS_TO_TICKS(20));
    if (more > 0) lastRawN += more;
    char b[3 * 24 + 1] = "";
    for (int i = 0; i < lastRawN && i < 24; i++) sprintf(b + 3 * i, "%02X ", lastRaw[i]);
    say("baud %7d: %d bytes: %s%s", baud, lastRawN, b, lastRawN > 6 ? " <- REPLY" : "");
  }
  say("bus reopen: %s", busOpen() ? "ok" : "FAILED");
}

static void holdAt(uint8_t id, int pos) { wrGoal(id, pos, 100, 0); }

// One guarded move. True if the target was reached.
static bool moveTo(uint8_t id, int target, int speed, int cap) {
  int trip = cap * 8, lastPos = -1, peak = 0, miss = 0, pos = 0, load, volt, temp;
  if (!wrGoal(id, target, speed, 50)) { say("  goal write failed"); return false; }
  uint32_t t0 = millis(), lastMove = millis();
  for (;;) {
    delay(20);
    if (Serial.available() && Serial.read() == 's') { holdAt(id, pos); say("  stopped by 's' at %d", pos); return false; }
    if (!readState(id, &pos, &load, &volt, &temp)) {
      if (++miss > 10) { wr8(id, R_TORQUE, 0); say("  LOST the servo: torque off sent"); return false; }
      continue;
    }
    miss = 0;
    if (abs(load) > peak) peak = abs(load);
    if (abs(load) >= trip) { holdAt(id, pos); say("  TRIP at %d: load %d (trip %d)", pos, load, trip); return false; }
    if (abs(pos - target) <= 8) { say("  reached %d (%.1f deg), peak load %d", pos, pos * 360.0 / 4096, peak); return true; }
    if (abs(pos - lastPos) > 2) { lastPos = pos; lastMove = millis(); }
    else if (millis() - lastMove > 400) { holdAt(id, pos); say("  BLOCKED at %d, %d short, load %d", pos, abs(target - pos), load); return false; }
    if (millis() - t0 > 15000) { holdAt(id, pos); say("  timeout at %d", pos); return false; }
  }
}

static void cmdSwing(uint8_t id, float degs, int cap, int speed) {
  int pos, load, volt, temp;
  uint8_t mode[1];
  cap = constrain(cap, 5, 40);
  speed = constrain(speed, 10, 400);
  if (fabs(degs) > 90) { say("refused: |deg| <= 90"); return; }
  if (!readState(id, &pos, &load, &volt, &temp) || !rd(id, R_MODE, 1, mode)) { say("id %d: no reply", id); return; }
  if (mode[0] != 0) { say("refused: mode %d, not position mode", mode[0]); return; }
  int d = (int)lround(degs * 4096 / 360);
  if (pos + abs(d) > 4095 || pos - abs(d) < 0) { say("refused: %d +/- %d crosses the 0/4095 wrap", pos, abs(d)); return; }
  say("id %d: start %d (%.1f deg), swing %+.1f deg (%d steps), cap %d%%, %d steps/s, %.1f V", id, pos,
      pos * 360.0 / 4096, degs, d, cap, speed, volt / 10.0);
  if (!wr16(id, R_TQ_LIMIT, cap * 10) || !wrGoal(id, pos, 100, 0) || !wr8(id, R_TORQUE, 1)) {
    say("setup write failed"); wr8(id, R_TORQUE, 0); return;
  }
  bool ok = moveTo(id, pos + d, speed, cap) && moveTo(id, pos, speed, cap) && moveTo(id, pos - d, speed, cap)
            && moveTo(id, pos, speed, cap);
  delay(200);
  wr8(id, R_TORQUE, 0);
  wr16(id, R_TQ_LIMIT, 1000);
  say("swing %s, torque off", ok ? "done" : "ended early");
}

static void handle(char *line) {
  char cmd[16] = "";
  float a = 0, b = 0, c = 0, d = 0;
  int n = sscanf(line, "%15s %f %f %f %f", cmd, &a, &b, &c, &d);
  if (n < 1) return;
  if (!strcmp(cmd, "bustest")) cmdBustest();
  else if (!strcmp(cmd, "low")) cmdLow(n >= 2 ? constrain((int)a, 1, 60) : 20);
  else if (!strcmp(cmd, "bauds")) cmdBauds();
  else if (!strcmp(cmd, "scan")) cmdScan(n >= 2 ? constrain((int)a, 0, 253) : 20);
  else if (!strcmp(cmd, "stat") && n >= 2) cmdStat((uint8_t)a);
  else if (!strcmp(cmd, "raw") && n >= 2) cmdRaw((uint8_t)a);
  else if (!strcmp(cmd, "swing") && n >= 3) cmdSwing((uint8_t)a, b, n >= 4 ? (int)c : 30, n >= 5 ? (int)d : 150);
  else if (!strcmp(cmd, "relax") && n >= 2) say("relax %d: %s", (int)a, wr8((uint8_t)a, R_TORQUE, 0) ? "ok" : "no reply");
  else say("? bustest | low [sec] | bauds | scan [maxid] | stat <id> | raw <id> | swing <id> <deg> [cap%%] [speed] | relax <id>");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  say("s3 buffered servo test ready (tx D7=GPIO%d, rx D6=GPIO%d, dir D8=GPIO%d): bus %s", PIN_TX, PIN_RX, PIN_DIR,
      busOpen() ? "open" : "FAILED");
  Serial.print("> ");
}

void loop() {
  static char line[80];
  static int len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { line[len] = 0; len = 0; handle(line); Serial.print("> "); }
    } else if (len < (int)sizeof line - 1) line[len++] = ch;
  }
}
