#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

namespace {
constexpr char kSsid[] = "Tether-Coexist-Test";
constexpr char kPassword[] = "tether-test";
constexpr uint8_t kChannel = 1;
constexpr uint8_t kBroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
constexpr uint8_t kNode1Mac[6] = {0x24, 0xEC, 0x4A, 0x3A, 0xEF, 0xB8};
bool espnow_started = false;
bool broadcast_added = false;
bool callback_registered = false;
bool node_peer_added = false;

void onReceive(const esp_now_recv_info_t *, const uint8_t *, int) {}

void printState() {
  wifi_mode_t mode = WIFI_MODE_NULL;
  uint8_t channel = 0;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  wifi_config_t config = {};
  const esp_err_t mode_result = esp_wifi_get_mode(&mode);
  const esp_err_t channel_result = esp_wifi_get_channel(&channel, &secondary);
  const esp_err_t config_result = esp_wifi_get_config(WIFI_IF_AP, &config);
  Serial.printf("STATE espnow=%u callback=%u node_peer=%u broadcast=%u "
                "mode=%d(%s) channel=%u(%s) "
                "hidden=%u(%s) stations=%u ip=%s ap_mac=%s\n",
                unsigned(espnow_started), unsigned(callback_registered),
                unsigned(node_peer_added), unsigned(broadcast_added), int(mode),
                esp_err_to_name(mode_result), channel, esp_err_to_name(channel_result),
                unsigned(config.ap.ssid_hidden), esp_err_to_name(config_result),
                unsigned(WiFi.softAPgetStationNum()), WiFi.softAPIP().toString().c_str(),
                WiFi.softAPmacAddress().c_str());
}

void registerCallback() {
  if (!espnow_started) { Serial.println("type n first"); return; }
  if (callback_registered) { Serial.println("callback already registered"); return; }
  const esp_err_t result = esp_now_register_recv_cb(onReceive);
  Serial.printf("esp_now_register_recv_cb -> %s\n", esp_err_to_name(result));
  callback_registered = result == ESP_OK;
  printState();
}

void addNodePeer() {
  if (!espnow_started) { Serial.println("type n first"); return; }
  if (node_peer_added) { Serial.println("node peer already added"); return; }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kNode1Mac, 6);
  peer.channel = kChannel;
  peer.ifidx = WIFI_IF_AP;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  Serial.printf("esp_now_add_peer(node1) -> %s\n", esp_err_to_name(result));
  node_peer_added = result == ESP_OK;
  printState();
}

void startEspNow() {
  if (espnow_started) { Serial.println("ESP-NOW already initialized"); return; }
  const esp_err_t result = esp_now_init();
  Serial.printf("esp_now_init -> %s\n", esp_err_to_name(result));
  espnow_started = result == ESP_OK;
  printState();
}

void addBroadcast() {
  if (!espnow_started) { Serial.println("type n first"); return; }
  if (broadcast_added) { Serial.println("broadcast peer already added"); return; }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kBroadcastMac, 6);
  peer.channel = kChannel;
  peer.ifidx = WIFI_IF_AP;
  peer.encrypt = false;
  const esp_err_t result = esp_now_add_peer(&peer);
  Serial.printf("esp_now_add_peer(broadcast) -> %s\n", esp_err_to_name(result));
  broadcast_added = result == ESP_OK;
  printState();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  const bool started = WiFi.softAP(kSsid, kPassword, kChannel, 0, 4);
  Serial.printf("softAP=%s SSID=%s password=%s\n",
                started ? "true" : "false", kSsid, kPassword);
  Serial.println("Confirm SSID after each step: n=init ESP-NOW, r=register RX callback,"
                 " p=add node-1 unicast peer, b=add broadcast peer, s=print state.");
  printState();
}

void loop() {
  if (Serial.available()) {
    const char command = char(Serial.read());
    if (command == 'n') startEspNow();
    else if (command == 'r') registerCallback();
    else if (command == 'p') addNodePeer();
    else if (command == 'b') addBroadcast();
    else if (command == 's') printState();
  }
  delay(10);
}
