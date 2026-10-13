#include "tether_link.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nvs_flash.h"

namespace tether {
namespace {
constexpr uint8_t kBroadcastMac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
constexpr size_t kRxQueueDepth = 24;

template <typename T>
void sealPacket(T &packet, PacketType type) {
  packet.header.magic = kMagic;
  packet.header.version = kVersion;
  packet.header.type = static_cast<uint8_t>(type);
  packet.header.size = sizeof(T);
  packet.checksum = 0;
  packet.checksum = checksum(&packet, sizeof(T));
}

template <typename T>
bool validPacket(const T &packet, size_t length, PacketType type) {
  if (length != sizeof(T) || packet.header.magic != kMagic ||
      packet.header.version != kVersion || packet.header.size != sizeof(T) ||
      packet.header.type != static_cast<uint8_t>(type)) return false;
  T copy = packet;
  const uint32_t expected = copy.checksum;
  copy.checksum = 0;
  return checksum(&copy, sizeof(copy)) == expected;
}
}  // namespace

EspNowLink *EspNowLink::instance_ = nullptr;

uint32_t checksum(const void *data, size_t length) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

void seal(Command &packet) { sealPacket(packet, PacketType::kCommand); }
void seal(Telemetry &packet) { sealPacket(packet, PacketType::kTelemetry); }
bool valid(const Command &packet, size_t length) {
  return validPacket(packet, length, PacketType::kCommand);
}
bool valid(const Telemetry &packet, size_t length) {
  return validPacket(packet, length, PacketType::kTelemetry);
}

esp_err_t EspNowLink::begin(uint8_t channel) {
  if (instance_) return ESP_ERR_INVALID_STATE;
  instance_ = this;
  queue_ = xQueueCreate(kRxQueueDepth, sizeof(ReceivedPacket));
  if (!queue_) return ESP_ERR_NO_MEM;

  // XIAO ESP32-C6 antenna switch: enable RF and select onboard antenna.
  gpio_config_t gpio = {};
  gpio.pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_14);
  gpio.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&gpio));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_3, 0));
  ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_14, 0));

  esp_err_t result = nvs_flash_init();
  if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
      result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    result = nvs_flash_init();
  }
  ESP_RETURN_ON_ERROR(result, "espnow", "NVS init failed");
  ESP_ERROR_CHECK(esp_netif_init());
  result = esp_event_loop_create_default();
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) return result;

  wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(receiveCallback));

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, kBroadcastMac, sizeof(kBroadcastMac));
  peer.channel = channel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));

  esp_now_rate_config_t rate = {};
  rate.phymode = WIFI_PHY_MODE_11G;
  rate.rate = WIFI_PHY_RATE_24M;
  ESP_ERROR_CHECK(esp_now_set_peer_rate_config(kBroadcastMac, &rate));
  return ESP_OK;
}

void EspNowLink::receiveCallback(const esp_now_recv_info_t *info,
                                 const uint8_t *data, int length) {
  if (!instance_ || !instance_->queue_ || !info || !data || length <= 0) return;
  ReceivedPacket packet = {};
  memcpy(packet.source_mac, info->src_addr, sizeof(packet.source_mac));
  packet.rssi_dbm = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  packet.length = static_cast<size_t>(length);
  const size_t capacity = sizeof(packet.payload);
  if (packet.length > capacity) return;
  memcpy(&packet.payload, data, packet.length);
  if (xQueueSend(static_cast<QueueHandle_t>(instance_->queue_), &packet, 0) != pdTRUE) {
    instance_->receive_drops_ = instance_->receive_drops_ + 1u;
  }
}

bool EspNowLink::receive(ReceivedPacket &packet, uint32_t timeout_ms) {
  return xQueueReceive(static_cast<QueueHandle_t>(queue_), &packet,
                       pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

esp_err_t EspNowLink::sendBytes(const void *data, size_t length) {
  return esp_now_send(kBroadcastMac, static_cast<const uint8_t *>(data), length);
}
esp_err_t EspNowLink::send(const Command &packet) {
  return sendBytes(&packet, sizeof(packet));
}
esp_err_t EspNowLink::send(const Telemetry &packet) {
  return sendBytes(&packet, sizeof(packet));
}

}  // namespace tether
