#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_twai.h"
#include "hal/gpio_types.h"

namespace damiao {

struct Limits {
  float position_rad = 12.5f;
  float velocity_rad_s = 20.0f;
  float torque_nm = 200.0f;
  bool from_motor = false;
};

struct Feedback {
  float position_rad = 0;
  float velocity_rad_s = 0;
  float torque_nm = 0;
  uint8_t error = 0;
  uint8_t mos_temperature_c = 0;
  uint8_t rotor_temperature_c = 0;
  uint32_t frames = 0;
  uint32_t last_frame_ms = 0;
};

struct BusHealth {
  twai_error_state_t state = TWAI_ERROR_ACTIVE;
  uint16_t tx_error_count = 0;
  uint16_t rx_error_count = 0;
  uint32_t bus_errors = 0;
  uint32_t rx_frames = 0;
  uint32_t tx_frames = 0;
  uint32_t tx_failures = 0;
  uint32_t rx_drops = 0;
};

class Motor {
 public:
  Motor(uint16_t slave_id, uint16_t master_id)
      : slave_id_(slave_id), master_id_(master_id) {}

  esp_err_t begin(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate = 1000000);
  void poll(size_t max_frames = 24);
  void serviceBusRecovery();

  bool enable();
  bool disable();
  bool setZeroPosition();
  bool refreshStatus();
  bool setControlMode(uint8_t mode);
  bool commandPositionVelocity(float position_rad, float velocity_rad_s);
  bool commandMit(float kp, float kd, float position_rad,
                  float velocity_rad_s, float torque_nm);
  bool syncLimitsFromMotor();

  const Feedback &feedback() const { return feedback_; }
  const Limits &limits() const { return limits_; }
  BusHealth health() const;
  bool seenRecently(uint32_t maximum_age_ms = 500) const;

 private:
  struct RxFrame {
    uint32_t id;
    uint8_t length;
    uint8_t data[8];
  };

  static bool receiveCallback(twai_node_handle_t handle,
                              const twai_rx_done_event_data_t *event,
                              void *context);
  bool send(uint32_t id, const uint8_t data[8]);
  bool controlByte(uint8_t value);
  bool decode(const RxFrame &frame);
  bool readFloatRegister(uint8_t register_id, float &value);
  static uint16_t floatToUint(float value, float minimum, float maximum,
                              uint8_t bits);
  static float uintToFloat(uint16_t value, float minimum, float maximum,
                           uint8_t bits);

  uint16_t slave_id_;
  uint16_t master_id_;
  twai_node_handle_t node_ = nullptr;
  void *rx_queue_ = nullptr;
  volatile uint32_t rx_drops_ = 0;
  uint32_t rx_frames_ = 0;
  uint32_t tx_frames_ = 0;
  uint32_t tx_failures_ = 0;
  Feedback feedback_;
  Limits limits_;
};

}  // namespace damiao
