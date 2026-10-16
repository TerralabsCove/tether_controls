#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_arduino_version.h>
#include <esp_idf_version.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "Config.h"
#include "PeerConfig.h"
#include "Protocol.h"
#include "DamiaoMotor.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 1, 0)
#error "Use Arduino-ESP32 4.0.0-RC1 or newer for the supported esp_twai.h API"
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "Select an ESP32-S3 or ESP32-C6 board"
#endif
// C5 migration: replace the guard above with:
// #if !defined(CONFIG_IDF_TARGET_ESP32S3) && \
//     !defined(CONFIG_IDF_TARGET_ESP32C6) && \
//     !defined(CONFIG_IDF_TARGET_ESP32C5)
// #error "Select an ESP32-S3, ESP32-C6, or ESP32-C5 board"
// #endif
#if TETHER_ROLE == TETHER_ROLE_MASTER && TETHER_NODE_ID != 0
#error "The master must be id 0"
#endif
#if TETHER_ROLE == TETHER_ROLE_NODE && TETHER_NODE_ID == 0
#error "A remote node cannot use id 0; id 0 is the master with a local motor"
#endif

namespace {
constexpr uint8_t kBroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
union Payload {
  tether::Header header;
  tether::Hello hello;
  tether::Command command;
  tether::Telemetry telemetry;
};
struct ReceivedPacket {
  size_t length;
  int8_t rssi;
  uint8_t source_mac[6];
  Payload payload;
};
QueueHandle_t radio_queue;
volatile uint32_t radio_drops = 0;

void printMac(const uint8_t mac[6]) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5]);
}

bool addPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = TETHER_ESPNOW_CHANNEL;
#if TETHER_ROLE == TETHER_ROLE_MASTER
  peer.ifidx = WIFI_IF_AP;
#else
  peer.ifidx = WIFI_IF_STA;
#endif
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int length) {
  if (!info || !data || length < int(sizeof(tether::Header)) ||
      length > int(sizeof(Payload))) return;
  ReceivedPacket packet = {};
  packet.length = size_t(length);
  packet.rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  memcpy(packet.source_mac, info->src_addr, 6);
  memcpy(&packet.payload, data, packet.length);
  if (xQueueSend(radio_queue, &packet, 0) != pdTRUE)
    radio_drops = radio_drops + 1u;
}

bool radioBegin() {
  radio_queue = xQueueCreate(24, sizeof(ReceivedPacket));
  if (!radio_queue) return false;
#if defined(CONFIG_IDF_TARGET_ESP32C6)
  // XIAO ESP32-C6 RF switch: enable RF and select the onboard antenna.
  pinMode(3, OUTPUT); digitalWrite(3, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, LOW);
#endif
#if TETHER_ROLE == TETHER_ROLE_MASTER
  WiFi.mode(WIFI_AP);
  // C5 5 GHz migration: enable immediately after WiFi.mode(), before softAP().
  // #if defined(CONFIG_IDF_TARGET_ESP32C5)
  // if (!WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY)) return false;
  // #endif
  WiFi.setSleep(false);
  // softAP owns the shared AP/STA radio channel. Resetting the channel after
  // AP startup can interrupt beaconing on some Arduino-ESP32/IDF builds.
  if (!WiFi.softAP(TETHER_MASTER_AP_SSID, TETHER_MASTER_AP_PASSWORD,
                   TETHER_ESPNOW_CHANNEL, 0, 4)) return false;
  delay(100);
  // Keep discovery identical to the proven AP smoke test. ESP-NOW and UDP
  // start only after a laptop has associated with this SoftAP.
  return true;
#else
  WiFi.mode(WIFI_STA);
  // C5 5 GHz migration: enable immediately after WiFi.mode(), before setting
  // the ESP-NOW channel. All C5 nodes and the master's SoftAP must match.
  // #if defined(CONFIG_IDF_TARGET_ESP32C5)
  // if (!WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY)) return false;
  // #endif
  WiFi.setSleep(false);
  if (esp_wifi_set_channel(TETHER_ESPNOW_CHANNEL,
                           WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK || esp_now_register_recv_cb(onReceive) != ESP_OK)
    return false;
  return addPeer(kBroadcastMac);
#endif
}

void printRadioState() {
  wifi_mode_t mode = WIFI_MODE_NULL;
  uint8_t channel = 0;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  const esp_err_t mode_result = esp_wifi_get_mode(&mode);
  const esp_err_t channel_result = esp_wifi_get_channel(&channel, &secondary);
#if TETHER_ROLE == TETHER_ROLE_MASTER
  wifi_config_t config = {};
  const esp_err_t config_result = esp_wifi_get_config(WIFI_IF_AP, &config);
  Serial.printf("radio mode=%d(%s) channel=%u(%s) AP_hidden=%u(%s) AP_MAC=%s\n",
                int(mode), esp_err_to_name(mode_result), channel,
                esp_err_to_name(channel_result), unsigned(config.ap.ssid_hidden),
                esp_err_to_name(config_result), WiFi.softAPmacAddress().c_str());
#else
  Serial.printf("radio mode=%d(%s) channel=%u(%s)\n", int(mode),
                esp_err_to_name(mode_result), channel,
                esp_err_to_name(channel_result));
#endif
}

template <typename T> bool sendPacket(const uint8_t mac[6], const T &packet) {
  return esp_now_send(mac, reinterpret_cast<const uint8_t *>(&packet),
                      sizeof(packet)) == ESP_OK;
}

void haltConfiguration(const char *reason, const uint8_t actual[6],
                       const char *interface_name = "station") {
  Serial.print("CONFIG ERROR: "); Serial.println(reason);
  Serial.printf("this ESP %s MAC = {", interface_name);
  for (size_t i = 0; i < 6; i++)
    Serial.printf("0x%02X%s", actual[i], i == 5 ? "" : ", ");
  Serial.println("}");
  while (true) delay(1000);
}

struct MotorRuntime {
  bool have_command = false;
  bool estop = true;
  bool enabled = false;
  bool watchdog = true;
  uint8_t previous_actions = 0;
  uint16_t last_sequence = 0;
  uint16_t telemetry_sequence = 0;
  uint32_t last_command_us = 0;
  uint32_t next_probe_ms = 0;
  uint32_t next_control_ms = 0;
  uint32_t next_telemetry_ms = 0;
};

DamiaoMotor motors[tether::kMotorCount] = {
    DamiaoMotor(TETHER_MOTOR_0_SLAVE_ID, TETHER_MOTOR_0_FEEDBACK_ID),
    DamiaoMotor(TETHER_MOTOR_1_SLAVE_ID, TETHER_MOTOR_1_FEEDBACK_ID),
    DamiaoMotor(TETHER_MOTOR_2_SLAVE_ID, TETHER_MOTOR_2_FEEDBACK_ID),
};
MotorRuntime motor_state[tether::kMotorCount];
bool motor_bus_ready = false;

const char *motorName(uint8_t node_id, uint8_t motor_index) {
  if (node_id == tether::kMasterId) return "master";
  if (motor_index == tether::kMotor0) return "big";
  if (motor_index == tether::kMotor1) return "near-master";
  return "far-end";
}

uint8_t motorCountForNode(uint8_t node_id) {
  return node_id == tether::kMasterId ? 1 : tether::kMotorCount;
}

bool localMotorIdsConfigured() {
  if (!TETHER_MOTORS_CONFIGURED) return false;
  for (uint8_t m = 0; m < TETHER_LOCAL_MOTOR_COUNT; m++)
    if (!motors[m].slaveId() || !motors[m].feedbackId()) return false;
  return true;
}

bool beginLocalMotors() {
  for (uint8_t m = 0; m < TETHER_LOCAL_MOTOR_COUNT; m++)
    if (!motors[m].begin((gpio_num_t)TETHER_CAN_TX_GPIO,
                         (gpio_num_t)TETHER_CAN_RX_GPIO,
                         TETHER_CAN_BITRATE)) return false;
  return true;
}

#if TETHER_ROLE == TETHER_ROLE_MASTER
struct Intent {
  uint8_t mode = tether::kIdle, flags = 0, reset_cycles = 0, zero_cycles = 0;
  float q = 0, dq = 0.5f, tau = 0, kp = 0, kd = 0;
};
struct Presence {
  uint32_t last_seen_ms = 0;
  int8_t rssi = 0;
  uint8_t motor_count = 0;
  uint32_t hello_count = 0;
};
Intent intents[3][tether::kMotorCount];
Presence presence[3];
uint16_t sequences[3][tether::kMotorCount] = {};
uint32_t next_command_ms = 0;
WiFiUDP control_udp;
IPAddress laptop_ip;
uint16_t laptop_port = 0;
bool have_laptop = false;
bool laptop_watchdog_latched = false;
uint32_t last_laptop_ms = 0;
bool master_services_started = false;
bool master_services_failed = false;

bool startMasterServices() {
  if (master_services_started) return true;
  if (master_services_failed || WiFi.softAPgetStationNum() == 0) return false;
  esp_err_t result = esp_now_init();
  if (result == ESP_OK) result = esp_now_register_recv_cb(onReceive);
  if (result != ESP_OK) {
    Serial.printf("MASTER SERVICE ERROR: ESP-NOW start failed: %s\n",
                  esp_err_to_name(result));
    master_services_failed = true;
    return false;
  }
  for (size_t i = 0; i < kNodePeerCount; i++) {
    if (!macIsZero(kNodePeers[i].mac) && !addPeer(kNodePeers[i].mac)) {
      Serial.printf("MASTER SERVICE ERROR: could not add node %u peer\n",
                    unsigned(kNodePeers[i].id));
      master_services_failed = true;
      return false;
    }
  }
  if (!control_udp.begin(TETHER_CONTROL_UDP_PORT)) {
    Serial.println("MASTER SERVICE ERROR: could not start UDP control port");
    master_services_failed = true;
    return false;
  }
  master_services_started = true;
  next_command_ms = millis();
  Serial.println("master services ready; ESP-NOW and UDP started after laptop association");
  return true;
}

String macString(const uint8_t mac[6]) {
  char text[18];
  snprintf(text, sizeof(text), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(text);
}

void sendLaptop(const String &line) {
  if (!have_laptop) return;
  control_udp.beginPacket(laptop_ip, laptop_port);
  control_udp.print(line);
  control_udp.endPacket();
}

void reportLine(const String &line) {
  Serial.println(line);
  sendLaptop(line);
}

void printPeers() {
  const uint32_t now = millis();
  for (uint8_t m = 0; m < TETHER_LOCAL_MOTOR_COUNT; m++) {
    const auto &feedback = motors[m].feedback();
    const auto &limits = motors[m].limits();
    reportLine("LOCAL id=0 motor=" + String(m) + " name=" + motorName(0, m) +
               " state=" + String(motors[m].seen() ? "SEEN" : "NOT_SEEN") +
               " enabled=" + String(motor_state[m].enabled ? "yes" : "no") +
               " estop=" + String(motor_state[m].estop ? "yes" : "no") +
               " limits=" + String(limits.valid ? "valid" : "unknown") +
               " motor_status=" + String(feedback.error));
  }
  for (size_t i = 0; i < kNodePeerCount; i++) {
    const auto &peer = kNodePeers[i];
    const bool online = presence[peer.id].last_seen_ms &&
                        now - presence[peer.id].last_seen_ms <= TETHER_PEER_OFFLINE_MS;
    String line = "PEER id=" + String(peer.id) + " name=" + peer.name +
                  " configured=" + (macIsZero(peer.mac) ? "no" : "yes") +
                  " state=" + (online ? "ONLINE" : "OFFLINE") + " age_ms=";
    line += presence[peer.id].last_seen_ms
                ? String(now - presence[peer.id].last_seen_ms) : String("never");
    line += " rssi=" + String(presence[peer.id].rssi) +
            " motors=" + String(presence[peer.id].motor_count) +
            " mac=" + macString(peer.mac);
    reportLine(line);
  }
}

void parseOperatorCommand(String line) {
  line.trim();
  if (line == "ping") {
    sendLaptop("PONG uptime_ms=" + String(millis()));
    return;
  }
  if (line == "peers" || line == "status") { printPeers(); return; }
  if (line == "help") {
    reportLine("HELP peers | reset N M | enable N M | disable N M | estop | zero N M | pos N M q dq | mit N M q dq tau kp kd; id 0: M=0, ids 1/2: M=0 big, 1 near, 2 far");
    return;
  }
  if (line == "estop") {
    for (uint8_t id = 0; id < 3; id++)
      for (uint8_t m = 0; m < motorCountForNode(id); m++) {
        intents[id][m].flags |= tether::kEstop;
        intents[id][m].flags &= ~tether::kEnable;
      }
    reportLine("ACK estop");
    return;
  }
  int id = -1, motor_index = -1; float a = 0, b = 0, c = 0, d = 0, e = 0;
  const auto known = [&]() {
    return id == 0 || (id > 0 && id < 3 && peerForId(uint8_t(id)));
  };
  const auto motorKnown = [&]() {
    return motor_index >= 0 && motor_index < motorCountForNode(uint8_t(id));
  };
  if (sscanf(line.c_str(), "reset %d %d", &id, &motor_index) == 2 && known() && motorKnown()) {
    auto &x = intents[id][motor_index]; x.reset_cycles = 5;
    x.flags &= ~(tether::kEnable | tether::kEstop);
    reportLine("ACK reset " + String(id) + " " + String(motor_index)); return;
  }
  if (sscanf(line.c_str(), "enable %d %d", &id, &motor_index) == 2 && known() && motorKnown()) {
    intents[id][motor_index].flags |= tether::kEnable;
    reportLine("ACK enable " + String(id) + " " + String(motor_index)); return;
  }
  if (sscanf(line.c_str(), "disable %d %d", &id, &motor_index) == 2 && known() && motorKnown()) {
    intents[id][motor_index].flags &= ~tether::kEnable;
    reportLine("ACK disable " + String(id) + " " + String(motor_index)); return;
  }
  if (sscanf(line.c_str(), "zero %d %d", &id, &motor_index) == 2 && known() && motorKnown()) {
    intents[id][motor_index].zero_cycles = 5;
    reportLine("ACK zero " + String(id) + " " + String(motor_index)); return;
  }
  if (sscanf(line.c_str(), "pos %d %d %f %f", &id, &motor_index, &a, &b) == 4 && known() && motorKnown()) {
    auto &x = intents[id][motor_index]; x.mode = tether::kPositionVelocity; x.q = a; x.dq = b;
    reportLine("ACK pos " + String(id) + " " + String(motor_index)); return;
  }
  if (sscanf(line.c_str(), "mit %d %d %f %f %f %f %f", &id, &motor_index, &a, &b, &c, &d, &e) == 7 && known() && motorKnown()) {
    auto &x = intents[id][motor_index]; x.mode = tether::kMit; x.q = a; x.dq = b;
    x.tau = c; x.kp = d; x.kd = e;
    reportLine("ACK mit " + String(id) + " " + String(motor_index)); return;
  }
  reportLine("ERR bad command; type help");
}

void pollLaptop() {
  int packet_size = control_udp.parsePacket();
  while (packet_size > 0) {
    char buffer[192];
    const int length = control_udp.read(buffer, sizeof(buffer) - 1);
    if (length > 0) {
      buffer[length] = 0;
      laptop_ip = control_udp.remoteIP();
      laptop_port = control_udp.remotePort();
      have_laptop = true;
      last_laptop_ms = millis();
      laptop_watchdog_latched = false;
      parseOperatorCommand(String(buffer));
    }
    packet_size = control_udp.parsePacket();
  }
  // USB serial remains an optional diagnostic input, never a requirement.
  if (Serial.available()) {
    last_laptop_ms = millis();
    laptop_watchdog_latched = false;
    parseOperatorCommand(Serial.readStringUntil('\n'));
  }
}

void serviceLaptopWatchdog(uint32_t now) {
  if (!last_laptop_ms || now - last_laptop_ms <= TETHER_LAPTOP_WATCHDOG_MS ||
      laptop_watchdog_latched) return;
  for (uint8_t id = 0; id < 3; id++)
    for (uint8_t m = 0; m < motorCountForNode(id); m++) {
      intents[id][m].flags |= tether::kEstop;
      intents[id][m].flags &= ~tether::kEnable;
    }
  laptop_watchdog_latched = true;
  reportLine("WATCHDOG laptop link lost; all motors estopped");
}

void applyLocalIntent(uint8_t motor_index, uint32_t now_us) {
  auto &intent = intents[0][motor_index];
  auto &state = motor_state[motor_index];
  auto &motor = motors[motor_index];
  const uint8_t flags = intent.flags |
      (intent.reset_cycles ? tether::kResetEstop : 0) |
      (intent.zero_cycles ? tether::kZeroPosition : 0);
  if (flags & tether::kEstop) state.estop = true;
  const uint8_t actions = flags & (tether::kResetEstop | tether::kZeroPosition);
  if ((actions & tether::kResetEstop) &&
      !(state.previous_actions & tether::kResetEstop) &&
      !(flags & tether::kEnable)) state.estop = false;
  if ((actions & tether::kZeroPosition) &&
      !(state.previous_actions & tether::kZeroPosition) &&
      !state.enabled && !state.estop) motor.zero();
  state.previous_actions = actions;
  state.have_command = true;
  state.last_command_us = now_us;
  sequences[0][motor_index]++;
  if (intent.reset_cycles) --intent.reset_cycles;
  if (intent.zero_cycles) --intent.zero_cycles;
}

void serviceLocalMotors() {
  if (!motor_bus_ready) return;
  motors[0].poll(48);
  motors[0].recover();
  const uint32_t now_ms = millis(), now_us = micros();
  for (uint8_t m = 0; m < TETHER_LOCAL_MOTOR_COUNT; m++) {
    auto &motor = motors[m]; auto &state = motor_state[m]; auto &intent = intents[0][m];
    if (!state.enabled && int32_t(now_ms - state.next_probe_ms) >= 0) {
      motor.refresh(); motor.poll(48);
      if (motor.seen() && !motor.limits().valid) motor.syncLimits();
      state.next_probe_ms = now_ms + 200;
    }
    state.watchdog = !state.have_command ||
        (now_us - state.last_command_us) > TETHER_COMMAND_WATCHDOG_US;
    CanHealth bus = motor.health();
    const bool bus_off = bus.state == TWAI_ERROR_BUS_OFF;
    const bool motor_fault = motor.feedback().error >= 8;
    if (bus_off || motor_fault || (state.enabled && !motor.seen())) state.estop = true;
    const bool wants_enable = state.have_command &&
        (intent.flags & tether::kEnable) && intent.mode != tether::kIdle;
    if (state.estop || state.watchdog || !wants_enable) {
      if (state.enabled) { motor.disable(); state.enabled = false; }
    } else if (!state.enabled && motor.limits().valid && motor.seen()) {
      if (motor.setControlMode(intent.mode == tether::kMit ? 1 : 2)) {
        delay(20); state.enabled = motor.enable();
      }
    }
    if (state.enabled && !state.estop && !state.watchdog &&
        int32_t(now_ms - state.next_control_ms) >= 0) {
      if (intent.mode == tether::kPositionVelocity) motor.posVel(intent.q, intent.dq);
      else if (intent.mode == tether::kMit)
        motor.mit(intent.q, intent.dq, intent.tau, intent.kp, intent.kd);
      state.next_control_ms = now_ms + 10;
    }
    if (int32_t(now_ms - state.next_telemetry_ms) >= 0) {
      const auto &f = motor.feedback(); bus = motor.health();
      const uint16_t flags = (state.enabled ? tether::kNodeEnabled : 0) |
          (state.watchdog ? tether::kNodeWatchdog : 0) |
          (state.estop ? tether::kNodeEstop : 0) |
          (bus.state == TWAI_ERROR_ACTIVE ? tether::kNodeCanActive : 0) |
          (motor.seen() ? tether::kNodeMotorSeen : 0) |
          (motor.limits().valid ? tether::kNodeLimitsValid : 0) |
          (motor_fault ? tether::kNodeMotorFault : 0) |
          (bus_off ? tether::kNodeBusOff : 0);
      char line[300];
      state.telemetry_sequence++;
      snprintf(line, sizeof(line),
               "TEL id=0 motor=%u name=%s seq=%u q=%.3f dq=%.3f tau=%.2f flags=%04x "
               "motor_status=%u temp=%u/%u rssi=0 can_rx=%lu can_tx=%lu fail=%lu buserr=%lu",
               m, motorName(0, m), state.telemetry_sequence, f.q, f.dq, f.tau, flags, f.error,
               f.mos_c, f.rotor_c, (unsigned long)bus.rx_frames,
               (unsigned long)bus.tx_frames, (unsigned long)bus.tx_failures,
               (unsigned long)bus.bus_errors);
      sendLaptop(String(line));
      if ((state.telemetry_sequence % 25u) == 0) Serial.println(line);
      state.next_telemetry_ms = now_ms + TETHER_TELEMETRY_PERIOD_MS;
    }
  }
}

void processMasterRadio() {
  ReceivedPacket packet;
  while (xQueueReceive(radio_queue, &packet, 0) == pdTRUE) {
    const uint8_t id = packet.payload.header.source_id;
    if (id == 0) continue;
    const PeerDefinition *configured = peerForId(id);
    if (packet.payload.header.type == tether::kHello &&
        packet.length == sizeof(tether::Hello) &&
        tether::valid(packet.payload.hello, packet.length, tether::kHello)) {
      if (!configured || macIsZero(configured->mac)) {
        String line = "DISCOVERED unconfigured id=" + String(id) + " mac={";
        for (size_t i = 0; i < 6; i++)
          line += "0x" + String(packet.source_mac[i], HEX) + (i == 5 ? "" : ", ");
        line += "}";
        reportLine(line);
        continue;
      }
      if (!macEqual(configured->mac, packet.source_mac)) {
        reportLine("ID CONFLICT id=" + String(id) + " expected=" +
                   macString(configured->mac) + " received=" + macString(packet.source_mac));
        continue;
      }
      const uint8_t advertised_motor_count =
          uint8_t((packet.payload.hello.capabilities >> 8) & 0xffu);
      if (advertised_motor_count != motorCountForNode(id)) {
        reportLine("CONFIG MISMATCH id=" + String(id) + " expected_motors=" +
                   String(motorCountForNode(id)) + " reported_motors=" +
                   String(advertised_motor_count));
        continue;
      }
      presence[id].last_seen_ms = millis(); presence[id].rssi = packet.rssi;
      presence[id].motor_count = advertised_motor_count;
      presence[id].hello_count++;
      continue;
    }
    if (!configured || macIsZero(configured->mac) ||
        !macEqual(configured->mac, packet.source_mac)) continue;
    if (packet.payload.header.type == tether::kTelemetry &&
        packet.length == sizeof(tether::Telemetry) &&
        tether::valid(packet.payload.telemetry, packet.length, tether::kTelemetry)) {
      const auto &t = packet.payload.telemetry;
      if (t.motor_index >= motorCountForNode(id)) continue;
      presence[id].last_seen_ms = millis(); presence[id].rssi = packet.rssi;
      char line[280];
      snprintf(line, sizeof(line),
               "TEL id=%u motor=%u name=%s seq=%u q=%.3f dq=%.3f tau=%.2f flags=%04x "
               "motor_status=%u temp=%u/%u rssi=%d can_rx=%lu can_tx=%lu fail=%lu buserr=%lu",
               id, t.motor_index, motorName(id, t.motor_index), t.header.sequence,
               t.position_rad, t.velocity_rad_s, t.torque_nm,
               t.node_flags, t.motor_error, t.mos_temperature_c,
               t.rotor_temperature_c, packet.rssi,
               (unsigned long)t.can_rx_frames, (unsigned long)t.can_tx_frames,
               (unsigned long)t.can_tx_failures, (unsigned long)t.can_bus_errors);
      sendLaptop(String(line));
      if ((t.header.sequence % 25u) == 0) {
        Serial.printf("id=%u motor=%u q=%.3f dq=%.3f tau=%.2f flags=%04x rssi=%d can_rx=%lu can_tx=%lu fail=%lu\n",
                      id, t.motor_index, t.position_rad, t.velocity_rad_s, t.torque_nm,
                      t.node_flags, packet.rssi, (unsigned long)t.can_rx_frames,
                      (unsigned long)t.can_tx_frames, (unsigned long)t.can_tx_failures);
      }
    }
  }
}

void masterLoop() {
  serviceLocalMotors();
  if (!master_services_started) {
    startMasterServices();
    return;
  }
  pollLaptop();
  processMasterRadio();
  const uint32_t now = millis();
  serviceLaptopWatchdog(now);
  if (int32_t(now - next_command_ms) < 0) return;
  next_command_ms += TETHER_COMMAND_PERIOD_MS;
  // Do not saturate the shared SoftAP/ESP-NOW radio with failed unicast
  // retries during discovery. Commands are useful only when both the laptop
  // control link and the destination node are currently alive.
  const bool laptop_online = have_laptop && last_laptop_ms &&
      now - last_laptop_ms <= TETHER_LAPTOP_WATCHDOG_MS;
  if (!laptop_online) return;
  for (uint8_t m = 0; m < TETHER_LOCAL_MOTOR_COUNT; m++) applyLocalIntent(m, micros());
  for (size_t i = 0; i < kNodePeerCount; i++) {
    const auto &peer = kNodePeers[i];
    const bool node_online = presence[peer.id].last_seen_ms &&
        now - presence[peer.id].last_seen_ms <= TETHER_PEER_OFFLINE_MS;
    if (macIsZero(peer.mac) || !node_online) continue;
    for (uint8_t m = 0; m < motorCountForNode(peer.id); m++) {
      auto &intent = intents[peer.id][m];
      tether::Command command = {};
      command.header.source_id = 0; command.header.target_id = peer.id;
      sequences[peer.id][m]++;
      command.header.sequence = sequences[peer.id][m]; command.header.sent_us = micros();
      command.motor_index = m; command.mode = intent.mode;
      command.flags = intent.flags |
          (intent.reset_cycles ? tether::kResetEstop : 0) |
          (intent.zero_cycles ? tether::kZeroPosition : 0);
      command.position_rad = intent.q; command.velocity_rad_s = intent.dq;
      command.torque_nm = intent.tau; command.kp = intent.kp; command.kd = intent.kd;
      tether::seal(command, tether::kCommand);
      sendPacket(peer.mac, command);
      if (intent.reset_cycles) --intent.reset_cycles;
      if (intent.zero_cycles) --intent.zero_cycles;
    }
  }
}

#else
bool identity_configured = false;
tether::Command commands[tether::kMotorCount] = {};
uint32_t commands_received[tether::kMotorCount] = {};
uint32_t command_gaps[tether::kMotorCount] = {};
int8_t command_rssi[tether::kMotorCount] = {};
uint16_t hello_sequence = 0;
uint32_t next_hello_ms = 0;

void sendHello() {
  tether::Hello hello = {};
  hello.header.source_id = TETHER_NODE_ID; hello.header.target_id = tether::kBroadcastId;
  hello_sequence++;
  hello.header.sequence = hello_sequence; hello.header.sent_us = micros();
  hello.uptime_ms = millis();
  hello.capabilities = 1u | (uint32_t(TETHER_LOCAL_MOTOR_COUNT) << 8);  // TWAI + motor count
  tether::seal(hello, tether::kHello); sendPacket(kBroadcastMac, hello);
}

void processNodeRadio() {
  ReceivedPacket packet;
  while (xQueueReceive(radio_queue, &packet, 0) == pdTRUE) {
    if (!identity_configured || !macEqual(packet.source_mac, kMasterApMac) ||
        packet.length != sizeof(tether::Command) ||
        !tether::valid(packet.payload.command, packet.length, tether::kCommand)) continue;
    const auto &candidate = packet.payload.command;
    if (candidate.header.source_id != 0 || candidate.header.target_id != TETHER_NODE_ID) continue;
    if (candidate.motor_index >= TETHER_LOCAL_MOTOR_COUNT) continue;
    const uint8_t m = candidate.motor_index;
    auto &state = motor_state[m];
    if (state.have_command) {
      const int16_t delta = int16_t(candidate.header.sequence - state.last_sequence);
      if (delta <= 0) continue;
      if (delta > 1) command_gaps[m] += uint16_t(delta - 1);
    }
    commands[m] = candidate; state.last_sequence = candidate.header.sequence;
    state.have_command = true; state.last_command_us = micros(); command_rssi[m] = packet.rssi;
    commands_received[m]++;
    if (candidate.flags & tether::kEstop) state.estop = true;
    const uint8_t actions = candidate.flags & (tether::kResetEstop | tether::kZeroPosition);
    if ((actions & tether::kResetEstop) && !(state.previous_actions & tether::kResetEstop) &&
        !(candidate.flags & tether::kEnable)) state.estop = false;
    if ((actions & tether::kZeroPosition) && !(state.previous_actions & tether::kZeroPosition) &&
        !state.enabled && !state.estop && motor_bus_ready) motors[m].zero();
    state.previous_actions = actions;
  }
}

void nodeLoop() {
  if (motor_bus_ready) { motors[0].poll(48); motors[0].recover(); }
  processNodeRadio();
  const uint32_t now_ms = millis(), now_us = micros();
  for (uint8_t m = 0; m < TETHER_LOCAL_MOTOR_COUNT; m++) {
    auto &motor = motors[m]; auto &state = motor_state[m]; auto &command = commands[m];
    if (motor_bus_ready && !state.enabled && int32_t(now_ms - state.next_probe_ms) >= 0) {
      motor.refresh(); motor.poll(48);
      if (motor.seen() && !motor.limits().valid) motor.syncLimits();
      state.next_probe_ms = now_ms + 200;
    }
    state.watchdog = !state.have_command ||
        (now_us - state.last_command_us) > TETHER_COMMAND_WATCHDOG_US;
    CanHealth bus = motor.health();
    const bool bus_off = motor_bus_ready && bus.state == TWAI_ERROR_BUS_OFF;
    const bool motor_fault = motor.feedback().error >= 8;
    if (bus_off || motor_fault || (state.enabled && !motor.seen())) state.estop = true;
    const bool wants_enable = motor_bus_ready && state.have_command &&
        (command.flags & tether::kEnable) && command.mode != tether::kIdle;
    if (state.estop || state.watchdog || !wants_enable) {
      if (state.enabled) { motor.disable(); state.enabled = false; }
    } else if (!state.enabled && motor.limits().valid && motor.seen()) {
      if (motor.setControlMode(command.mode == tether::kMit ? 1 : 2)) {
        delay(20); state.enabled = motor.enable();
      }
    }
    if (state.enabled && !state.estop && !state.watchdog &&
        int32_t(now_ms - state.next_control_ms) >= 0) {
      if (command.mode == tether::kPositionVelocity)
        motor.posVel(command.position_rad, command.velocity_rad_s);
      else if (command.mode == tether::kMit)
        motor.mit(command.position_rad, command.velocity_rad_s, command.torque_nm,
                  command.kp, command.kd);
      state.next_control_ms = now_ms + 10;
    }
    if (int32_t(now_ms - state.next_telemetry_ms) >= 0) {
      const auto &f = motor.feedback(); const auto &l = motor.limits(); bus = motor.health();
      tether::Telemetry t = {};
      t.header.source_id = TETHER_NODE_ID; t.header.target_id = 0;
      state.telemetry_sequence++;
      t.header.sequence = state.telemetry_sequence; t.header.sent_us = now_us;
      t.last_command_sequence = state.last_sequence; t.motor_index = m;
      t.motor_slave_id = motor.slaveId(); t.motor_feedback_id = motor.feedbackId();
      t.node_flags = (state.enabled ? tether::kNodeEnabled : 0) |
          (state.watchdog ? tether::kNodeWatchdog : 0) | (state.estop ? tether::kNodeEstop : 0) |
          (motor_bus_ready && bus.state == TWAI_ERROR_ACTIVE ? tether::kNodeCanActive : 0) |
          (motor.seen() ? tether::kNodeMotorSeen : 0) |
          (l.valid ? tether::kNodeLimitsValid : 0) |
          (motor_fault ? tether::kNodeMotorFault : 0) |
          (bus_off ? tether::kNodeBusOff : 0);
      t.command_age_us = state.have_command ? now_us - state.last_command_us : UINT32_MAX;
      t.uptime_ms = now_ms; t.position_rad = f.q; t.velocity_rad_s = f.dq; t.torque_nm = f.tau;
      t.position_limit_rad = l.q; t.velocity_limit_rad_s = l.dq; t.torque_limit_nm = l.tau;
      t.motor_error = f.error; t.mos_temperature_c = f.mos_c; t.rotor_temperature_c = f.rotor_c;
      t.command_rssi_dbm = command_rssi[m]; t.can_tx_error_count = bus.tx_error;
      t.can_rx_error_count = bus.rx_error; t.can_bus_errors = bus.bus_errors;
      t.can_rx_frames = bus.rx_frames; t.can_tx_frames = bus.tx_frames;
      t.can_tx_failures = bus.tx_failures; t.command_received = commands_received[m];
      t.command_gaps = command_gaps[m]; t.radio_rx_drops = radio_drops;
      tether::seal(t, tether::kTelemetry); sendPacket(kMasterApMac, t);
      state.next_telemetry_ms = now_ms + TETHER_TELEMETRY_PERIOD_MS;
    }
  }
  if (int32_t(now_ms - next_hello_ms) >= 0) {
    sendHello(); next_hello_ms = now_ms + TETHER_HELLO_PERIOD_MS;
  }
}
#endif
}  // namespace

void setup() {
  Serial.begin(115200); delay(500);
  // Read the permanent station identity without starting the station radio.
  // The master must enter AP mode directly; an STA -> AP transition prevented
  // SoftAP beaconing on the S3/Arduino-ESP32 4.0.0-RC1 bench.
  uint8_t own_mac[6] = {};
  if (esp_read_mac(own_mac, ESP_MAC_WIFI_STA) != ESP_OK) {
    Serial.println("CONFIG ERROR: could not read factory Wi-Fi station MAC");
    while (true) delay(1000);
  }
  Serial.printf("Tether role=%d id=%d Arduino=%s IDF=%s MAC=", TETHER_ROLE,
                TETHER_NODE_ID, ESP_ARDUINO_VERSION_STR, esp_get_idf_version());
  printMac(own_mac); Serial.println();
#if TETHER_ROLE == TETHER_ROLE_MASTER
  if (macIsZero(kMasterMac) || !macEqual(own_mac, kMasterMac))
    haltConfiguration("set kMasterMac in PeerConfig.h before using id 0", own_mac);
#else
  const PeerDefinition *self = peerForId(TETHER_NODE_ID);
  if (!self || macIsZero(kMasterMac) || macIsZero(kMasterApMac))
    haltConfiguration("configure both master MACs and this node id first", own_mac);
  if (macIsZero(self->mac)) {
    Serial.println("commissioning mode: broadcasting this node MAC; motor commands are blocked");
  } else if (!macEqual(own_mac, self->mac)) {
    haltConfiguration("configured node MAC does not match this ESP", own_mac);
  } else {
    identity_configured = true;
  }
#endif
  if (!radioBegin()) haltConfiguration("ESP-NOW initialization failed", own_mac);
  printRadioState();
#if TETHER_ROLE == TETHER_ROLE_MASTER
  uint8_t ap_mac[6] = {};
  if (esp_wifi_get_mac(WIFI_IF_AP, ap_mac) != ESP_OK ||
      macIsZero(kMasterApMac) || !macEqual(ap_mac, kMasterApMac))
    haltConfiguration("set kMasterApMac in PeerConfig.h before using id 0",
                      ap_mac, "AP");
  Serial.printf("master AP=%s channel=%u IP=%s UDP=%u\n",
                TETHER_MASTER_AP_SSID, TETHER_ESPNOW_CHANNEL,
                WiFi.softAPIP().toString().c_str(), TETHER_CONTROL_UDP_PORT);
  if (!localMotorIdsConfigured())
    haltConfiguration("configure the id-0 motor before flashing the master", own_mac);
  motor_bus_ready = beginLocalMotors();
  if (!motor_bus_ready) haltConfiguration("master local TWAI/motor initialization failed", own_mac);
  for (auto &state : motor_state)
    state.next_probe_ms = state.next_control_ms = state.next_telemetry_ms = millis();
  Serial.println("master AP and local motor ready; estop latched; waiting for laptop association");
#else
  if (!addPeer(kMasterApMac)) haltConfiguration("could not register master AP peer", own_mac);
  if (TETHER_MOTORS_CONFIGURED) {
    if (!localMotorIdsConfigured())
      haltConfiguration("configure all three node motor IDs", own_mac);
    motor_bus_ready = beginLocalMotors();
    if (!motor_bus_ready) haltConfiguration("TWAI/three-motor initialization failed", own_mac);
  } else {
    Serial.println("COMMISSIONING: motor IDs unconfigured; radio online, motor control blocked");
  }
  for (auto &state : motor_state)
    state.next_probe_ms = state.next_control_ms = state.next_telemetry_ms = millis();
  next_hello_ms = millis();
  Serial.println("node ready; all three motors estopped");
#endif
}

void loop() {
#if TETHER_ROLE == TETHER_ROLE_MASTER
  masterLoop();
#else
  nodeLoop();
#endif
  delay(1);
}
