#include <Arduino.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_idf_version.h>
#include <esp_wifi.h>

namespace {
constexpr char kSsid[] = "Tether-AP-Test";
constexpr char kPassword[] = "tether-test";
constexpr uint8_t kChannel = 1;
// C5 5 GHz test: replace the line above with this after selecting the
// XIAO_ESP32C5 board and confirming the channel is permitted locally.
// constexpr uint8_t kChannel = 36;

void printRadioState() {
  wifi_mode_t mode = WIFI_MODE_NULL;
  uint8_t primary = 0;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  int8_t power = 0;
  wifi_config_t config = {};
  const esp_err_t mode_error = esp_wifi_get_mode(&mode);
  const esp_err_t channel_error = esp_wifi_get_channel(&primary, &secondary);
  const esp_err_t power_error = esp_wifi_get_max_tx_power(&power);
  const esp_err_t config_error = esp_wifi_get_config(WIFI_IF_AP, &config);
  Serial.printf(
      "AP_DIAG mode=%d mode_err=%s channel=%u channel_err=%s tx_power_qdbm=%d "
      "power_err=%s hidden=%u config_err=%s stations=%u ip=%s ap_mac=%s\n",
      int(mode), esp_err_to_name(mode_error), primary, esp_err_to_name(channel_error),
      int(power), esp_err_to_name(power_error), unsigned(config.ap.ssid_hidden),
      esp_err_to_name(config_error), unsigned(WiFi.softAPgetStationNum()),
      WiFi.softAPIP().toString().c_str(), WiFi.softAPmacAddress().c_str());
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("AP smoke test: Arduino=%s IDF=%s\n",
                ESP_ARDUINO_VERSION_STR, esp_get_idf_version());
  WiFi.mode(WIFI_AP);
  // C5 5 GHz test: enable after WiFi.mode() and before WiFi.softAP().
  // #if defined(CONFIG_IDF_TARGET_ESP32C5)
  // if (!WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY)) {
  //   Serial.println("Could not select the 5 GHz Wi-Fi band");
  //   return;
  // }
  // #endif
  WiFi.setSleep(false);
  const bool started = WiFi.softAP(kSsid, kPassword, kChannel, 0, 4);
  Serial.printf("softAP returned %s; look for SSID %s password %s\n",
                started ? "true" : "false", kSsid, kPassword);
  printRadioState();
}

void loop() {
  static uint32_t next_print_ms = 0;
  if (int32_t(millis() - next_print_ms) >= 0) {
    printRadioState();
    next_print_ms = millis() + 2000;
  }
  delay(10);
}
