#include <Arduino.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_wifi.h>

namespace {
void printInitializer(const char *label, const uint8_t mac[6]) {
  Serial.printf("%s: %02X:%02X:%02X:%02X:%02X:%02X\n", label,
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("%s initializer: {", label);
  for (size_t i = 0; i < 6; ++i)
    Serial.printf("0x%02X%s", mac[i], i == 5 ? "" : ", ");
  Serial.println("}");
}
}

void setup() {
  Serial.begin(115200);
  delay(500);

#if defined(CONFIG_IDF_TARGET_ESP32C6)
  // XIAO ESP32-C6 RF switch: enable RF and select the onboard antenna.
  pinMode(3, OUTPUT); digitalWrite(3, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, LOW);
#endif

  uint8_t station_mac[6] = {}, softap_mac[6] = {};
  if (esp_read_mac(station_mac, ESP_MAC_WIFI_STA) != ESP_OK ||
      esp_read_mac(softap_mac, ESP_MAC_WIFI_SOFTAP) != ESP_OK) {
    Serial.println("Could not derive Wi-Fi MAC addresses");
    return;
  }
  printInitializer("Station MAC", station_mac);
  printInitializer("SoftAP MAC", softap_mac);
  Serial.println("MASTER: kMasterMac=Station MAC; kMasterApMac=SoftAP MAC.");
  Serial.println("NODE: use Station MAC in the matching kNodePeers row.");
  Serial.println("IMPORTANT: nodes send ESP-NOW traffic to the master's SoftAP MAC.");
  Serial.printf("Flash bytes: %lu; PSRAM bytes: %lu\n",
                static_cast<unsigned long>(ESP.getFlashChipSize()),
                static_cast<unsigned long>(ESP.getPsramSize()));
}

void loop() { delay(1000); }
