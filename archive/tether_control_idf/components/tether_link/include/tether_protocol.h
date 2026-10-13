#pragma once

#include <stddef.h>
#include <stdint.h>

namespace tether {

constexpr uint32_t kMagic = 0x32524854u;  // "THR2" little-endian
constexpr uint8_t kVersion = 1;
constexpr uint8_t kBroadcastNode = 0xff;

enum class PacketType : uint8_t { kCommand = 1, kTelemetry = 2 };
enum class ControlMode : uint8_t { kIdle = 0, kPositionVelocity = 1, kMit = 2 };
enum CommandFlags : uint8_t {
  kEnable = 1u << 0,
  kEstop = 1u << 1,
  kResetEstop = 1u << 2,
  kZeroPosition = 1u << 3,
};
enum NodeFlags : uint16_t {
  kNodeEnabled = 1u << 0,
  kNodeWatchdog = 1u << 1,
  kNodeEstop = 1u << 2,
  kNodeCanActive = 1u << 3,
  kNodeMotorSeen = 1u << 4,
  kNodeLimitsValid = 1u << 5,
  kNodeMotorFault = 1u << 6,
  kNodeBusOff = 1u << 7,
};

struct __attribute__((packed)) Header {
  uint32_t magic;
  uint8_t version;
  uint8_t type;
  uint8_t source_node;
  uint8_t target_node;
  uint16_t sequence;
  uint16_t size;
  uint32_t sent_us;
};

// Commands are state, not events. The coordinator repeats the entire desired
// state, so loss of one ESP-NOW packet cannot lose an enable/stop transition.
struct __attribute__((packed)) Command {
  Header header;
  uint8_t mode;
  uint8_t flags;
  uint16_t reserved;
  float position_rad;
  float velocity_rad_s;
  float torque_nm;
  float kp;
  float kd;
  uint32_t checksum;
};

struct __attribute__((packed)) Telemetry {
  Header header;
  uint16_t last_command_sequence;
  uint16_t node_flags;
  uint32_t command_age_us;
  uint32_t uptime_ms;
  float position_rad;
  float velocity_rad_s;
  float torque_nm;
  float position_limit_rad;
  float velocity_limit_rad_s;
  float torque_limit_nm;
  uint8_t motor_error;
  uint8_t mos_temperature_c;
  uint8_t rotor_temperature_c;
  int8_t command_rssi_dbm;
  uint16_t can_tx_error_count;
  uint16_t can_rx_error_count;
  uint32_t can_bus_errors;
  uint32_t can_rx_frames;
  uint32_t can_tx_frames;
  uint32_t can_tx_failures;
  uint32_t command_received;
  uint32_t command_gaps;
  uint32_t radio_rx_drops;
  uint32_t checksum;
};

uint32_t checksum(const void *data, size_t length);
void seal(Command &packet);
void seal(Telemetry &packet);
bool valid(const Command &packet, size_t received_length);
bool valid(const Telemetry &packet, size_t received_length);

static_assert(sizeof(Command) <= 250, "ESP-NOW v1 payload limit exceeded");
static_assert(sizeof(Telemetry) <= 250, "ESP-NOW v1 payload limit exceeded");

}  // namespace tether
