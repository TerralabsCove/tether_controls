#include <Arduino.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_mac.h>

#if !defined(CONFIG_IDF_TARGET_ESP32C6)
#error "Select XIAO_ESP32C6 in Arduino IDE"
#endif

namespace {
void printMac(const char *name, const uint8_t mac[6]) {
  Serial.printf("%s=%02X:%02X:%02X:%02X:%02X:%02X\n", name,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("%s initializer={", name);
  for (size_t i = 0; i < 6; i++)
    Serial.printf("0x%02X%s", mac[i], i == 5 ? "" : ", ");
  Serial.println("}");
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(750);

  // XIAO ESP32-C6 RF switch: enable RF and select the onboard antenna.
  pinMode(3, OUTPUT); digitalWrite(3, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, LOW);

  uint8_t station[6] = {}, softap[6] = {};
  const esp_err_t sta_result = esp_read_mac(station, ESP_MAC_WIFI_STA);
  const esp_err_t ap_result = esp_read_mac(softap, ESP_MAC_WIFI_SOFTAP);

  Serial.printf("chip=XIAO_ESP32C6 Arduino=%s IDF=%s\n",
                ESP_ARDUINO_VERSION_STR, esp_get_idf_version());
  if (sta_result != ESP_OK || ap_result != ESP_OK) {
    Serial.printf("MAC read failed: station=%s softap=%s\n",
                  esp_err_to_name(sta_result), esp_err_to_name(ap_result));
    return;
  }
  printMac("station_mac", station);
  printMac("softap_mac", softap);
  Serial.println("master kMasterMac <- station_mac");
  Serial.println("master kMasterApMac <- softap_mac (REQUIRED ESP-NOW peer address)");
  Serial.println("node kNodePeers row <- station_mac");
}

void loop() { delay(1000); }
