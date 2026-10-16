#include <Arduino.h>
#include <esp_idf_version.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 1, 0)
#error "Use Arduino-ESP32 4.0.0-RC1 or newer for the supported TWAI node API"
#endif

#include <esp_twai.h>
#include <esp_twai_onchip.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
constexpr gpio_num_t kTx = GPIO_NUM_17;
constexpr gpio_num_t kRx = GPIO_NUM_18;
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
constexpr gpio_num_t kTx = GPIO_NUM_20;  // XIAO D9 -> transceiver TXD
constexpr gpio_num_t kRx = GPIO_NUM_18;  // XIAO D10 <- transceiver RXD
#else
#error "Select an ESP32-S3 or XIAO ESP32-C6 board"
#endif
constexpr uint32_t kBitrate = 1000000;
twai_node_handle_t node;
QueueHandle_t rx_queue;

struct RxFrame { uint32_t id; uint8_t length; uint8_t data[8]; };

bool IRAM_ATTR onRx(twai_node_handle_t handle,
                    const twai_rx_done_event_data_t *, void *) {
  RxFrame item = {};
  twai_frame_t frame = {};
  frame.buffer = item.data;
  frame.buffer_len = sizeof(item.data);
  if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) return false;
  item.id = frame.header.id;
  item.length = twaifd_dlc2len(frame.header.dlc);
  BaseType_t woken = pdFALSE;
  xQueueSendFromISR(rx_queue, &item, &woken);
  return woken == pdTRUE;
}

bool sendProbe() {
  uint8_t data[8] = {'T', 'E', 'T', 'H', 'E', 'R', 0x01, 0x00};
  twai_frame_t frame = {};
  frame.header.id = 0x123;
  frame.header.dlc = 8;
  frame.buffer = data;
  frame.buffer_len = sizeof(data);
  return twai_node_transmit(node, &frame, 20) == ESP_OK &&
         twai_node_transmit_wait_all_done(node, 20) == ESP_OK;
}

void setup() {
  Serial.begin(115200); delay(500);
#if defined(CONFIG_IDF_TARGET_ESP32C6)
  pinMode(3, OUTPUT); digitalWrite(3, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, LOW);
#endif
  rx_queue = xQueueCreate(32, sizeof(RxFrame));
  twai_onchip_node_config_t config = {};
  config.io_cfg.tx = kTx; config.io_cfg.rx = kRx;
  config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  config.bit_timing.bitrate = kBitrate;
  config.tx_queue_depth = 8;
  twai_event_callbacks_t callbacks = {};
  callbacks.on_rx_done = onRx;
  if (!rx_queue || twai_new_node_onchip(&config, &node) != ESP_OK ||
      twai_node_register_event_callbacks(node, &callbacks, nullptr) != ESP_OK ||
      twai_node_enable(node) != ESP_OK) {
    Serial.println("TWAI setup failed"); while (true) delay(1000);
  }
  Serial.println("1 Mbit/s CAN ready. Type s to send standard id 0x123.");
}

void loop() {
  if (Serial.read() == 's') Serial.println(sendProbe() ? "TX ACKed" : "TX failed/no ACK");
  RxFrame frame;
  while (xQueueReceive(rx_queue, &frame, 0) == pdTRUE) {
    Serial.printf("RX %03lX [%u]", static_cast<unsigned long>(frame.id), frame.length);
    for (uint8_t i = 0; i < frame.length; i++) Serial.printf(" %02X", frame.data[i]);
    Serial.println();
  }
  twai_node_status_t status;
  if (twai_node_get_info(node, &status, nullptr) == ESP_OK &&
      status.state == TWAI_ERROR_BUS_OFF) {
    Serial.println("BUS OFF; requesting recovery"); twai_node_recover(node);
  }
  delay(1);
}
