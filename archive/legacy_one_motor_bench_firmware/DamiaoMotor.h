#pragma once

#include <Arduino.h>
#include <esp_idf_version.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 1, 0)
#error "Install Arduino-ESP32 4.0.0-RC1 or newer; DamiaoMotor requires the supported esp_twai.h API from ESP-IDF 6.1"
#endif

#include <esp_twai.h>

struct MotorLimits { float q = 12.5f, dq = 20.0f, tau = 200.0f; bool valid = false; };
struct MotorFeedback {
  float q = 0, dq = 0, tau = 0; uint8_t error = 0, mos_c = 0, rotor_c = 0;
  uint32_t frames = 0, last_ms = 0;
};
struct CanHealth {
  twai_error_state_t state = TWAI_ERROR_ACTIVE;
  uint16_t tx_error = 0, rx_error = 0; uint32_t bus_errors = 0;
  uint32_t rx_frames = 0, tx_frames = 0, tx_failures = 0, rx_drops = 0;
};

class DamiaoMotor {
 public:
  DamiaoMotor(uint16_t slave, uint16_t master) : slave_(slave), master_(master) {}
  bool begin(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate = 1000000);
  void poll(size_t maximum = 24); void recover(); CanHealth health() const;
  bool enable(); bool disable(); bool zero(); bool refresh(); bool syncLimits();
  bool setControlMode(uint8_t mode);
  bool posVel(float q, float dq); bool mit(float q, float dq, float tau, float kp, float kd);
  bool seen(uint32_t age_ms = 500) const;
  const MotorLimits &limits() const { return limits_; }
  const MotorFeedback &feedback() const { return feedback_; }
 private:
  struct Rx { uint32_t id; uint8_t length, data[8]; };
  static bool rxCallback(twai_node_handle_t, const twai_rx_done_event_data_t *, void *);
  bool send(uint32_t id, const uint8_t data[8]); bool control(uint8_t byte);
  bool decode(const Rx &); bool readRegister(uint8_t id, float &value);
  static uint16_t pack(float x, float lo, float hi, uint8_t bits);
  static float unpack(uint16_t x, float lo, float hi, uint8_t bits);
  uint16_t slave_, master_; twai_node_handle_t node_ = nullptr; QueueHandle_t queue_ = nullptr;
  volatile uint32_t drops_ = 0; uint32_t rx_ = 0, tx_ = 0, tx_fail_ = 0;
  MotorLimits limits_; MotorFeedback feedback_;
};
