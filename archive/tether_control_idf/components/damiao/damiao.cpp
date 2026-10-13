#include "damiao.h"

#include <algorithm>
#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace damiao {
namespace {
constexpr uint32_t kBroadcastId = 0x7ff;
constexpr uint32_t kPositionVelocityOffset = 0x100;
constexpr uint32_t kTxTimeoutMs = 20;
constexpr uint8_t kRegisterPmax = 21;
constexpr uint8_t kRegisterVmax = 22;
constexpr uint8_t kRegisterTmax = 23;

uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
float clamp(float value, float low, float high) {
  return std::max(low, std::min(value, high));
}
}  // namespace

esp_err_t Motor::begin(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate) {
  rx_queue_ = xQueueCreate(64, sizeof(RxFrame));
  if (!rx_queue_) return ESP_ERR_NO_MEM;
  twai_onchip_node_config_t config = {};
  config.io_cfg.tx = tx;
  config.io_cfg.rx = rx;
  config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
  config.bit_timing.bitrate = bitrate;
  config.fail_retry_cnt = 3;
  config.tx_queue_depth = 16;
  config.flags.no_receive_rtr = true;
  esp_err_t result = twai_new_node_onchip(&config, &node_);
  if (result != ESP_OK) return result;
  twai_event_callbacks_t callbacks = {};
  callbacks.on_rx_done = receiveCallback;
  if ((result = twai_node_register_event_callbacks(node_, &callbacks, this)) != ESP_OK)
    return result;
  return twai_node_enable(node_);
}

bool IRAM_ATTR Motor::receiveCallback(twai_node_handle_t handle,
                                      const twai_rx_done_event_data_t *,
                                      void *context) {
  auto *self = static_cast<Motor *>(context);
  RxFrame item = {};
  twai_frame_t frame = {};
  frame.buffer = item.data;
  frame.buffer_len = sizeof(item.data);
  if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) return false;
  if (frame.header.ide || frame.header.rtr) return false;
  item.id = frame.header.id;
  item.length = static_cast<uint8_t>(twaifd_dlc2len(frame.header.dlc));
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(static_cast<QueueHandle_t>(self->rx_queue_), &item,
                       &woken) != pdTRUE) {
    self->rx_drops_ = self->rx_drops_ + 1u;
  }
  return woken == pdTRUE;
}

bool Motor::send(uint32_t id, const uint8_t data[8]) {
  uint8_t payload[8];
  memcpy(payload, data, sizeof(payload));
  twai_frame_t frame = {};
  frame.header.id = id;
  frame.header.dlc = 8;
  frame.buffer = payload;
  frame.buffer_len = sizeof(payload);
  if (twai_node_transmit(node_, &frame, kTxTimeoutMs) != ESP_OK ||
      twai_node_transmit_wait_all_done(node_, kTxTimeoutMs) != ESP_OK) {
    ++tx_failures_;
    return false;
  }
  ++tx_frames_;
  return true;
}

bool Motor::controlByte(uint8_t value) {
  const uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, value};
  return send(slave_id_, data);
}
bool Motor::enable() { return controlByte(0xfc); }
bool Motor::disable() { return controlByte(0xfd); }
bool Motor::setZeroPosition() { return controlByte(0xfe); }
bool Motor::refreshStatus() { return controlByte(0xcc); }
bool Motor::setControlMode(uint8_t mode) {
  const uint8_t data[8] = {static_cast<uint8_t>(slave_id_),
                           static_cast<uint8_t>(slave_id_ >> 8),
                           0x55, 10, mode, 0, 0, 0};
  return send(kBroadcastId, data);
}

bool Motor::commandPositionVelocity(float position, float velocity) {
  position = clamp(position, -limits_.position_rad, limits_.position_rad);
  velocity = clamp(fabsf(velocity), 0.0f, limits_.velocity_rad_s);
  uint8_t data[8];
  memcpy(&data[0], &position, 4);
  memcpy(&data[4], &velocity, 4);
  return send(slave_id_ + kPositionVelocityOffset, data);
}

bool Motor::commandMit(float kp, float kd, float position, float velocity,
                       float torque) {
  position = clamp(position, -limits_.position_rad, limits_.position_rad);
  velocity = clamp(velocity, -limits_.velocity_rad_s, limits_.velocity_rad_s);
  torque = clamp(torque, -limits_.torque_nm, limits_.torque_nm);
  kp = clamp(kp, 0.0f, 500.0f);
  kd = clamp(kd, 0.0f, 5.0f);
  const uint16_t p = floatToUint(position, -limits_.position_rad,
                                 limits_.position_rad, 16);
  const uint16_t v = floatToUint(velocity, -limits_.velocity_rad_s,
                                 limits_.velocity_rad_s, 12);
  const uint16_t k_p = floatToUint(kp, 0, 500, 12);
  const uint16_t k_d = floatToUint(kd, 0, 5, 12);
  const uint16_t t = floatToUint(torque, -limits_.torque_nm,
                                 limits_.torque_nm, 12);
  const uint8_t data[8] = {
      static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
      static_cast<uint8_t>(v >> 4),
      static_cast<uint8_t>((v & 0xf) << 4 | (k_p >> 8)),
      static_cast<uint8_t>(k_p), static_cast<uint8_t>(k_d >> 4),
      static_cast<uint8_t>((k_d & 0xf) << 4 | (t >> 8)),
      static_cast<uint8_t>(t),
  };
  return send(slave_id_, data);
}

void Motor::poll(size_t max_frames) {
  RxFrame frame;
  for (size_t i = 0; i < max_frames; ++i) {
    if (xQueueReceive(static_cast<QueueHandle_t>(rx_queue_), &frame, 0) != pdTRUE)
      break;
    ++rx_frames_;
    decode(frame);
  }
}

bool Motor::decode(const RxFrame &frame) {
  if (frame.length < 8) return false;
  if (frame.data[2] == 0x33 || frame.data[2] == 0x55) return false;
  if (frame.id != master_id_ && frame.id != slave_id_ && frame.id != 0) return false;
  const uint16_t p = (uint16_t(frame.data[1]) << 8) | frame.data[2];
  const uint16_t v = (uint16_t(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const uint16_t t = (uint16_t(frame.data[4] & 0xf) << 8) | frame.data[5];
  feedback_.position_rad = uintToFloat(p, -limits_.position_rad, limits_.position_rad, 16);
  feedback_.velocity_rad_s = uintToFloat(v, -limits_.velocity_rad_s, limits_.velocity_rad_s, 12);
  feedback_.torque_nm = uintToFloat(t, -limits_.torque_nm, limits_.torque_nm, 12);
  feedback_.error = frame.data[0] >> 4;
  feedback_.mos_temperature_c = frame.data[6];
  feedback_.rotor_temperature_c = frame.data[7];
  ++feedback_.frames;
  feedback_.last_frame_ms = nowMs();
  return true;
}

bool Motor::readFloatRegister(uint8_t register_id, float &value) {
  const uint8_t request[8] = {static_cast<uint8_t>(slave_id_),
                              static_cast<uint8_t>(slave_id_ >> 8),
                              0x33, register_id, 0, 0, 0, 0};
  if (!send(kBroadcastId, request)) return false;
  const uint32_t deadline = nowMs() + 100;
  RxFrame frame;
  while (static_cast<int32_t>(nowMs() - deadline) < 0) {
    if (xQueueReceive(static_cast<QueueHandle_t>(rx_queue_), &frame,
                      pdMS_TO_TICKS(5)) != pdTRUE) continue;
    ++rx_frames_;
    if (frame.length == 8 && frame.data[2] == 0x33 &&
        frame.data[3] == register_id &&
        ((uint16_t(frame.data[1]) << 8) | frame.data[0]) == slave_id_) {
      memcpy(&value, &frame.data[4], 4);
      return isfinite(value) && value > 0;
    }
    decode(frame);
  }
  return false;
}

bool Motor::syncLimitsFromMotor() {
  float p, v, t;
  if (!readFloatRegister(kRegisterPmax, p) ||
      !readFloatRegister(kRegisterVmax, v) ||
      !readFloatRegister(kRegisterTmax, t)) return false;
  limits_ = {.position_rad = p, .velocity_rad_s = v,
             .torque_nm = t, .from_motor = true};
  return true;
}

void Motor::serviceBusRecovery() {
  twai_node_status_t status;
  if (twai_node_get_info(node_, &status, nullptr) == ESP_OK &&
      status.state == TWAI_ERROR_BUS_OFF) (void)twai_node_recover(node_);
}

BusHealth Motor::health() const {
  BusHealth health = {};
  twai_node_status_t status;
  twai_node_record_t record;
  if (twai_node_get_info(node_, &status, &record) == ESP_OK) {
    health.state = status.state;
    health.tx_error_count = status.tx_error_count;
    health.rx_error_count = status.rx_error_count;
    health.bus_errors = record.bus_err_num;
  }
  health.rx_frames = rx_frames_;
  health.tx_frames = tx_frames_;
  health.tx_failures = tx_failures_;
  health.rx_drops = rx_drops_;
  return health;
}

bool Motor::seenRecently(uint32_t maximum_age_ms) const {
  return feedback_.frames && (nowMs() - feedback_.last_frame_ms) <= maximum_age_ms;
}

uint16_t Motor::floatToUint(float value, float minimum, float maximum,
                            uint8_t bits) {
  value = clamp(value, minimum, maximum);
  return static_cast<uint16_t>((value - minimum) * ((1u << bits) - 1u) /
                               (maximum - minimum));
}
float Motor::uintToFloat(uint16_t value, float minimum, float maximum,
                         uint8_t bits) {
  return static_cast<float>(value) * (maximum - minimum) /
             static_cast<float>((1u << bits) - 1u) + minimum;
}

}  // namespace damiao
