#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "../../firmware/tether_control_arduino/PeerConfig.h"
#include "../../firmware/tether_control_arduino/Protocol.h"

// Temporary transport: this ESP remains node 1 on ESP-NOW. USB serial only
// carries motor commands/feedback between the node and the laptop CAN bridge.
#define TETHER_NODE_ID 1

namespace {
// Must match TETHER_ESPNOW_CHANNEL in the canonical Arduino Config.h.
constexpr uint8_t kChannel = 1;
constexpr uint32_t kCommandWatchdogUs = 60000;
constexpr uint32_t kBridgeTimeoutMs = 750;
constexpr uint8_t kBroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

union Payload { tether::Header header; tether::Hello hello; tether::Command command; };
struct ReceivedPacket {
  size_t length;
  int8_t rssi;
  uint8_t source_mac[6];
  Payload payload;
};
QueueHandle_t radio_queue;
volatile uint32_t radio_drops = 0;

tether::Command command = {};
bool have_command = false;
uint16_t last_sequence = 0, telemetry_sequence = 0, hello_sequence = 0;
uint32_t last_command_us = 0, commands_received = 0, command_gaps = 0;
uint32_t next_telemetry_ms = 0, next_hello_ms = 0;
int8_t command_rssi = 0;
String serial_line;

struct BridgeState {
  bool connected = false, motor_seen = false, enabled = false, estop = true;
  uint8_t motor_index = 0;
  uint8_t error = 0, mos_c = 0, rotor_c = 0;
  float q = 0, dq = 0, tau = 0, q_limit = 0, dq_limit = 0, tau_limit = 0;
  uint32_t last_status_ms = 0, last_feedback_ms = 0, feedback_frames = 0;
} bridge;

bool addPeer(const uint8_t mac[6]) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = kChannel;
  peer.ifidx = WIFI_IF_STA;
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
  if (xQueueSend(radio_queue, &packet, 0) != pdTRUE) ++radio_drops;
}

template <typename T> bool sendPacket(const uint8_t mac[6], const T &packet) {
  return esp_now_send(mac, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)) == ESP_OK;
}

void emitBridgeCommand(const tether::Command &x) {
  Serial.printf("CMD,%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                unsigned(x.header.sequence), unsigned(x.flags), unsigned(x.mode),
                x.position_rad, x.velocity_rad_s, x.torque_nm, x.kp, x.kd);
}

void processRadio() {
  ReceivedPacket packet;
  while (xQueueReceive(radio_queue, &packet, 0) == pdTRUE) {
    if (!macEqual(packet.source_mac, kMasterApMac) ||
        packet.length != sizeof(tether::Command) ||
        !tether::valid(packet.payload.command, packet.length, tether::kCommand)) continue;
    const auto &candidate = packet.payload.command;
    if (candidate.header.source_id != 0 || candidate.header.target_id != TETHER_NODE_ID) continue;
    if (!bridge.connected || candidate.motor_index != bridge.motor_index) continue;
    if (have_command) {
      const int16_t delta = int16_t(candidate.header.sequence - last_sequence);
      if (delta <= 0) continue;
      if (delta > 1) command_gaps += uint16_t(delta - 1);
    }
    command = candidate;
    last_sequence = candidate.header.sequence;
    last_command_us = micros();
    command_rssi = packet.rssi;
    have_command = true;
    ++commands_received;
    emitBridgeCommand(command);
  }
}

void processBridgeLine(String line) {
  line.trim();
  int ready = 0, motor_index = 0;
  float q_limit = 0, dq_limit = 0, tau_limit = 0;
  if (sscanf(line.c_str(), "BRIDGE,%d,%d,%f,%f,%f", &ready, &motor_index,
             &q_limit, &dq_limit, &tau_limit) == 5 && motor_index >= 0 && motor_index < tether::kMotorCount) {
    bridge.connected = ready != 0;
    bridge.motor_index = uint8_t(motor_index);
    bridge.q_limit = q_limit; bridge.dq_limit = dq_limit; bridge.tau_limit = tau_limit;
    bridge.last_status_ms = millis();
    return;
  }
  unsigned seq = 0, seen = 0, enabled = 0, estop = 1, error = 0, mos = 0, rotor = 0;
  float q = 0, dq = 0, tau = 0;
  if (sscanf(line.c_str(), "TEL,%u,%u,%u,%u,%u,%f,%f,%f,%u,%u",
             &seq, &seen, &enabled, &estop, &error, &q, &dq, &tau, &mos, &rotor) == 10) {
    bridge.motor_seen = seen != 0;
    bridge.enabled = enabled != 0;
    bridge.estop = estop != 0;
    bridge.error = uint8_t(error);
    bridge.q = q; bridge.dq = dq; bridge.tau = tau;
    bridge.mos_c = uint8_t(mos); bridge.rotor_c = uint8_t(rotor);
    bridge.last_feedback_ms = millis();
    ++bridge.feedback_frames;
  }
}

void pollSerial() {
  while (Serial.available()) {
    const char c = char(Serial.read());
    if (c == '\n') { processBridgeLine(serial_line); serial_line = ""; }
    else if (c != '\r' && serial_line.length() < 180) serial_line += c;
  }
}

void sendHello() {
  tether::Hello hello = {};
  hello.header.source_id = TETHER_NODE_ID;
  hello.header.target_id = tether::kBroadcastId;
  hello.header.sequence = ++hello_sequence;
  hello.header.sent_us = micros();
  hello.uptime_ms = millis();
  hello.capabilities = 2u | (1u << 8);  // laptop serial transport for one selected slot
  tether::seal(hello, tether::kHello);
  sendPacket(kBroadcastMac, hello);
}

void sendTelemetry() {
  const uint32_t now_ms = millis(), now_us = micros();
  const bool watchdog = !have_command || now_us - last_command_us > kCommandWatchdogUs;
  const bool bridge_online = bridge.connected && now_ms - bridge.last_status_ms <= kBridgeTimeoutMs;
  const bool motor_seen = bridge.motor_seen && now_ms - bridge.last_feedback_ms <= 500;
  tether::Telemetry t = {};
  t.header.source_id = TETHER_NODE_ID;
  t.header.target_id = 0;
  t.header.sequence = ++telemetry_sequence;
  t.header.sent_us = now_us;
  t.last_command_sequence = last_sequence;
  t.motor_index = bridge.motor_index;
  t.node_flags = (bridge_online && bridge.enabled ? tether::kNodeEnabled : 0) |
      (watchdog ? tether::kNodeWatchdog : 0) |
      (!bridge_online || bridge.estop ? tether::kNodeEstop : 0) |
      (bridge_online ? tether::kNodeCanActive : 0) |
      (motor_seen ? tether::kNodeMotorSeen : 0) |
      (bridge_online && bridge.q_limit > 0 && bridge.dq_limit > 0 && bridge.tau_limit > 0
           ? tether::kNodeLimitsValid : 0) |
      (bridge.error >= 8 ? tether::kNodeMotorFault : 0);
  t.command_age_us = have_command ? now_us - last_command_us : UINT32_MAX;
  t.uptime_ms = now_ms;
  t.position_rad = bridge.q; t.velocity_rad_s = bridge.dq; t.torque_nm = bridge.tau;
  t.position_limit_rad = bridge.q_limit;
  t.velocity_limit_rad_s = bridge.dq_limit;
  t.torque_limit_nm = bridge.tau_limit;
  t.motor_error = bridge.error;
  t.mos_temperature_c = bridge.mos_c; t.rotor_temperature_c = bridge.rotor_c;
  t.command_rssi_dbm = command_rssi;
  t.can_rx_frames = bridge.feedback_frames; t.can_tx_frames = commands_received;
  t.command_received = commands_received; t.command_gaps = command_gaps;
  t.radio_rx_drops = radio_drops;
  tether::seal(t, tether::kTelemetry);
  sendPacket(kMasterApMac, t);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  uint8_t own_mac[6] = {};
  esp_wifi_get_mac(WIFI_IF_STA, own_mac);
  const PeerDefinition *self = peerForId(TETHER_NODE_ID);
  if (!self || macIsZero(kMasterMac) || macIsZero(kMasterApMac) ||
      macIsZero(self->mac) || !macEqual(own_mac, self->mac)) {
    Serial.printf("CONFIG_ERROR,node=%u,mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  TETHER_NODE_ID, own_mac[0], own_mac[1], own_mac[2], own_mac[3],
                  own_mac[4], own_mac[5]);
    while (true) delay(1000);
  }
  radio_queue = xQueueCreate(24, sizeof(ReceivedPacket));
  if (!radio_queue || esp_wifi_set_channel(kChannel, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK || esp_now_register_recv_cb(onReceive) != ESP_OK ||
      !addPeer(kBroadcastMac) || !addPeer(kMasterApMac)) {
    Serial.println("CONFIG_ERROR,espnow-init");
    while (true) delay(1000);
  }
  Serial.printf("HELLO,1,wireless-serial-node,id=%u\n", TETHER_NODE_ID);
  next_telemetry_ms = next_hello_ms = millis();
}

void loop() {
  processRadio();
  pollSerial();
  const uint32_t now = millis();
  if (int32_t(now - next_hello_ms) >= 0) { sendHello(); next_hello_ms += 1000; }
  if (int32_t(now - next_telemetry_ms) >= 0) { sendTelemetry(); next_telemetry_ms += 40; }
  delay(1);
}
