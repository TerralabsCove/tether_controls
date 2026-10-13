#pragma once

#include <Arduino.h>

namespace tether {
constexpr uint32_t kMagic = 0x33524854u;
constexpr uint8_t kVersion = 3;
constexpr uint8_t kMasterId = 0;
constexpr uint8_t kBroadcastId = 0xff;
constexpr uint8_t kMotorCount = 3;
enum MotorIndex : uint8_t { kSmallMotor = 0, kBigMotor = 1, kThirdMotor = 2 };

enum PacketType : uint8_t { kHello = 1, kCommand = 2, kTelemetry = 3 };
enum ControlMode : uint8_t { kIdle = 0, kPositionVelocity = 1, kMit = 2 };
enum CommandFlags : uint8_t {
  kEnable = 1u << 0, kEstop = 1u << 1,
  kResetEstop = 1u << 2, kZeroPosition = 1u << 3,
};
enum NodeFlags : uint16_t {
  kNodeEnabled = 1u << 0, kNodeWatchdog = 1u << 1,
  kNodeEstop = 1u << 2, kNodeCanActive = 1u << 3,
  kNodeMotorSeen = 1u << 4, kNodeLimitsValid = 1u << 5,
  kNodeMotorFault = 1u << 6, kNodeBusOff = 1u << 7,
};

struct __attribute__((packed)) Header {
  uint32_t magic;
  uint8_t version, type, source_id, target_id;
  uint16_t sequence, size;
  uint32_t sent_us;
};
struct __attribute__((packed)) Hello {
  Header header;
  uint32_t uptime_ms;
  uint32_t capabilities;
  uint32_t checksum;
};
struct __attribute__((packed)) Command {
  Header header;
  uint8_t motor_index, mode, flags, reserved;
  float position_rad, velocity_rad_s, torque_nm, kp, kd;
  uint32_t checksum;
};
struct __attribute__((packed)) Telemetry {
  Header header;
  uint16_t last_command_sequence, node_flags;
  uint8_t motor_index, reserved;
  uint16_t motor_slave_id, motor_feedback_id;
  uint32_t command_age_us, uptime_ms;
  float position_rad, velocity_rad_s, torque_nm;
  float position_limit_rad, velocity_limit_rad_s, torque_limit_nm;
  uint8_t motor_error, mos_temperature_c, rotor_temperature_c;
  int8_t command_rssi_dbm;
  uint16_t can_tx_error_count, can_rx_error_count;
  uint32_t can_bus_errors, can_rx_frames, can_tx_frames, can_tx_failures;
  uint32_t command_received, command_gaps, radio_rx_drops, checksum;
};

inline uint32_t checksum(const void *data, size_t length) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  uint32_t hash = 2166136261u;
  while (length--) { hash ^= *bytes++; hash *= 16777619u; }
  return hash;
}
template <typename T> inline void seal(T &packet, PacketType type) {
  packet.header.magic = kMagic;
  packet.header.version = kVersion;
  packet.header.type = type;
  packet.header.size = sizeof(T);
  packet.checksum = 0;
  packet.checksum = checksum(&packet, sizeof(T));
}
template <typename T> inline bool valid(const T &packet, size_t length, PacketType type) {
  if (length != sizeof(T) || packet.header.magic != kMagic ||
      packet.header.version != kVersion || packet.header.type != type ||
      packet.header.size != sizeof(T)) return false;
  T copy = packet;
  const uint32_t expected = copy.checksum;
  copy.checksum = 0;
  return checksum(&copy, sizeof(copy)) == expected;
}
static_assert(sizeof(Hello) <= 250 && sizeof(Command) <= 250 &&
              sizeof(Telemetry) <= 250, "ESP-NOW packet too large");
}  // namespace tether
