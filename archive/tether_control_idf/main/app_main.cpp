#include <stdio.h>
#include <string.h>

#include "damiao.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tether_link.h"

namespace {
constexpr int kRoleCoordinator = 0;
constexpr uint8_t kMotorNodeCount = 2;
constexpr uint32_t kCommandPeriodMs = 20;   // 50 Hz per motor node
constexpr uint32_t kTelemetryPeriodMs = 40; // 25 Hz per motor node
constexpr uint32_t kWatchdogMs = 60;
constexpr gpio_num_t kCanTx = GPIO_NUM_20;  // XIAO ESP32-C6 D9
constexpr gpio_num_t kCanRx = GPIO_NUM_18;  // XIAO ESP32-C6 D10
constexpr uint32_t kCanBitrate = 1000000;
constexpr uint8_t kEspNowChannel = 1;

uint32_t nowMs() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
uint32_t nowUs() { return static_cast<uint32_t>(esp_timer_get_time()); }

#if TETHER_ROLE == 0
struct OperatorState {
  tether::ControlMode mode = tether::ControlMode::kIdle;
  bool enable = false;
  bool estop = false;
  uint8_t reset_cycles = 0;
  uint8_t zero_cycles = 0;
  float position = 0;
  float velocity = 0.5f;
  float torque = 0;
  float kp = 0;
  float kd = 0;
};

class LocalMotorController {
 public:
  LocalMotorController() : motor_(TETHER_MOTOR_SLAVE_ID, TETHER_MOTOR_MASTER_ID) {}

  esp_err_t begin() {
    if (!TETHER_MASTER_MOTOR_CONFIGURED || TETHER_MOTOR_SLAVE_ID == 0 ||
        TETHER_MOTOR_MASTER_ID == 0) {
      ESP_LOGE("coordinator", "configure the id-0 motor CAN IDs before running");
      return ESP_ERR_INVALID_STATE;
    }
    const esp_err_t result = motor_.begin(kCanTx, kCanRx, kCanBitrate);
    next_probe_ = next_control_ = next_telemetry_ = nowMs();
    return result;
  }

  void apply(OperatorState &state, uint16_t sequence) {
    const uint8_t flags = (state.enable ? tether::kEnable : 0) |
                          (state.estop ? tether::kEstop : 0) |
                          (state.reset_cycles ? tether::kResetEstop : 0) |
                          (state.zero_cycles ? tether::kZeroPosition : 0);
    if (flags & tether::kEstop) estop_ = true;
    const uint8_t actions = flags & (tether::kResetEstop | tether::kZeroPosition);
    if ((actions & tether::kResetEstop) &&
        !(previous_actions_ & tether::kResetEstop) && !state.enable)
      estop_ = false;
    if ((actions & tether::kZeroPosition) &&
        !(previous_actions_ & tether::kZeroPosition) && !enabled_ && !estop_)
      (void)motor_.setZeroPosition();
    previous_actions_ = actions;
    state_ = state;
    last_sequence_ = sequence;
    last_command_us_ = nowUs();
    have_command_ = true;
    if (state.reset_cycles) --state.reset_cycles;
    if (state.zero_cycles) --state.zero_cycles;
  }

  void service(bool force_status) {
    motor_.poll();
    motor_.serviceBusRecovery();
    const uint32_t now_ms = nowMs(), now_us = nowUs();
    if (!enabled_ && static_cast<int32_t>(now_ms - next_probe_) >= 0) {
      (void)motor_.refreshStatus();
      motor_.poll();
      if (motor_.seenRecently() && !limits_valid_)
        limits_valid_ = motor_.syncLimitsFromMotor();
      next_probe_ = now_ms + 200;
    }

    const bool watchdog = !have_command_ ||
        (now_us - last_command_us_) > kWatchdogMs * 1000u;
    const auto bus = motor_.health();
    const bool bus_off = bus.state == TWAI_ERROR_BUS_OFF;
    const bool motor_fault = motor_.feedback().error >= 0x8;
    if (bus_off || motor_fault || (enabled_ && !motor_.seenRecently())) estop_ = true;
    const bool wants_enable = have_command_ && state_.enable &&
        state_.mode != tether::ControlMode::kIdle;
    if (estop_ || watchdog || !wants_enable) {
      if (enabled_) { (void)motor_.disable(); enabled_ = false; }
    } else if (!enabled_ && limits_valid_ && motor_.seenRecently()) {
      const uint8_t mode = state_.mode == tether::ControlMode::kMit ? 1 : 2;
      if (motor_.setControlMode(mode)) {
        vTaskDelay(pdMS_TO_TICKS(20));
        enabled_ = motor_.enable();
      }
    }
    if (enabled_ && !estop_ && !watchdog &&
        static_cast<int32_t>(now_ms - next_control_) >= 0) {
      if (state_.mode == tether::ControlMode::kPositionVelocity)
        (void)motor_.commandPositionVelocity(state_.position, state_.velocity);
      else if (state_.mode == tether::ControlMode::kMit)
        (void)motor_.commandMit(state_.kp, state_.kd, state_.position,
                                state_.velocity, state_.torque);
      next_control_ = now_ms + 10;
    }
    if (force_status || static_cast<int32_t>(now_ms - next_telemetry_) >= 0) {
      const auto &feedback = motor_.feedback();
      const auto current_bus = motor_.health();
      const uint16_t flags = (enabled_ ? tether::kNodeEnabled : 0) |
          (watchdog ? tether::kNodeWatchdog : 0) |
          (estop_ ? tether::kNodeEstop : 0) |
          (current_bus.state == TWAI_ERROR_ACTIVE ? tether::kNodeCanActive : 0) |
          (motor_.seenRecently() ? tether::kNodeMotorSeen : 0) |
          (limits_valid_ ? tether::kNodeLimitsValid : 0) |
          (motor_fault ? tether::kNodeMotorFault : 0) |
          (bus_off ? tether::kNodeBusOff : 0);
      printf("node=0 cmd=%u flags=0x%04x q=%.3f dq=%.3f tau=%.2f "
             "temp=%u/%uC motor_status=0x%x can=%lu/%lu fail=%lu buserr=%lu\n",
             last_sequence_, flags, feedback.position_rad, feedback.velocity_rad_s,
             feedback.torque_nm, feedback.mos_temperature_c,
             feedback.rotor_temperature_c, feedback.error,
             static_cast<unsigned long>(current_bus.rx_frames),
             static_cast<unsigned long>(current_bus.tx_frames),
             static_cast<unsigned long>(current_bus.tx_failures),
             static_cast<unsigned long>(current_bus.bus_errors));
      next_telemetry_ = now_ms + 1000;
    }
  }

 private:
  damiao::Motor motor_;
  OperatorState state_;
  bool estop_ = true, enabled_ = false, have_command_ = false;
  bool limits_valid_ = false;
  uint8_t previous_actions_ = 0;
  uint16_t last_sequence_ = 0;
  uint32_t last_command_us_ = 0;
  uint32_t next_probe_ = 0, next_control_ = 0, next_telemetry_ = 0;
};

enum class ConsoleAction { kStatus, kReset, kEnable, kDisable, kEstop, kZero, kPos, kMit };
struct ConsoleMessage {
  ConsoleAction action;
  uint8_t node;
  float values[5];
};
QueueHandle_t console_queue;

void consoleTask(void *) {
  printf("commands: status | reset <node> | enable <node> | disable <node> | "
         "estop | zero <node> | pos <node> <rad> <rad/s> | "
         "mit <node> <q> <dq> <tau> <kp> <kd>\n");
  char line[128];
  while (fgets(line, sizeof(line), stdin)) {
    ConsoleMessage message = {};
    unsigned node = 0;
    if (!strncmp(line, "status", 6)) {
      message.action = ConsoleAction::kStatus;
    } else if (sscanf(line, "reset %u", &node) == 1) {
      message.action = ConsoleAction::kReset; message.node = node;
    } else if (sscanf(line, "enable %u", &node) == 1) {
      message.action = ConsoleAction::kEnable; message.node = node;
    } else if (sscanf(line, "disable %u", &node) == 1) {
      message.action = ConsoleAction::kDisable; message.node = node;
    } else if (!strncmp(line, "estop", 5)) {
      message.action = ConsoleAction::kEstop; message.node = tether::kBroadcastNode;
    } else if (sscanf(line, "zero %u", &node) == 1) {
      message.action = ConsoleAction::kZero; message.node = node;
    } else if (sscanf(line, "pos %u %f %f", &node, &message.values[0],
                      &message.values[1]) == 3) {
      message.action = ConsoleAction::kPos; message.node = node;
    } else if (sscanf(line, "mit %u %f %f %f %f %f", &node,
                      &message.values[0], &message.values[1], &message.values[2],
                      &message.values[3], &message.values[4]) == 6) {
      message.action = ConsoleAction::kMit; message.node = node;
    } else {
      printf("unknown command\n");
      continue;
    }
    xQueueSend(console_queue, &message, portMAX_DELAY);
  }
  vTaskDelete(nullptr);
}

void coordinatorRun() {
  tether::EspNowLink link;
  ESP_ERROR_CHECK(link.begin(kEspNowChannel));
  LocalMotorController local_motor;
  ESP_ERROR_CHECK(local_motor.begin());
  console_queue = xQueueCreate(12, sizeof(ConsoleMessage));
  xTaskCreate(consoleTask, "console", 4096, nullptr, 4, nullptr);
  OperatorState state[kMotorNodeCount + 1] = {};
  uint16_t sequence[kMotorNodeCount + 1] = {};
  uint32_t next_command = nowMs();
  bool print_status = true;
  ESP_LOGI("coordinator", "ESP-NOW channel %u, 24 Mbps; safe idle at boot",
           kEspNowChannel);

  for (;;) {
    ConsoleMessage input;
    while (xQueueReceive(console_queue, &input, 0) == pdTRUE) {
      if (input.action == ConsoleAction::kStatus) { print_status = true; continue; }
      if (input.action == ConsoleAction::kEstop) {
        for (uint8_t n = 0; n <= kMotorNodeCount; ++n) {
          state[n].estop = true; state[n].enable = false;
        }
        continue;
      }
      if (input.node > kMotorNodeCount) {
        printf("node must be 0, 1, or 2\n"); continue;
      }
      auto &s = state[input.node];
      switch (input.action) {
        case ConsoleAction::kReset: s.reset_cycles = 5; s.estop = false; s.enable = false; break;
        case ConsoleAction::kEnable: s.enable = true; break;
        case ConsoleAction::kDisable: s.enable = false; break;
        case ConsoleAction::kZero: s.zero_cycles = 5; break;
        case ConsoleAction::kPos:
          s.mode = tether::ControlMode::kPositionVelocity;
          s.position = input.values[0]; s.velocity = input.values[1]; break;
        case ConsoleAction::kMit:
          s.mode = tether::ControlMode::kMit;
          s.position = input.values[0]; s.velocity = input.values[1];
          s.torque = input.values[2]; s.kp = input.values[3]; s.kd = input.values[4]; break;
        default: break;
      }
    }

    const uint32_t now = nowMs();
    if (static_cast<int32_t>(now - next_command) >= 0) {
      local_motor.apply(state[0], ++sequence[0]);
      for (uint8_t node = 1; node <= kMotorNodeCount; ++node) {
        auto &s = state[node];
        tether::Command command = {};
        command.header.source_node = 0;
        command.header.target_node = node;
        command.header.sequence = ++sequence[node];
        command.header.sent_us = nowUs();
        command.mode = static_cast<uint8_t>(s.mode);
        command.flags = (s.enable ? tether::kEnable : 0) |
                        (s.estop ? tether::kEstop : 0) |
                        (s.reset_cycles ? tether::kResetEstop : 0) |
                        (s.zero_cycles ? tether::kZeroPosition : 0);
        command.position_rad = s.position;
        command.velocity_rad_s = s.velocity;
        command.torque_nm = s.torque;
        command.kp = s.kp;
        command.kd = s.kd;
        tether::seal(command);
        (void)link.send(command);
        if (s.reset_cycles) --s.reset_cycles;
        if (s.zero_cycles) --s.zero_cycles;
      }
      next_command += kCommandPeriodMs;
    }

    local_motor.service(print_status);

    tether::ReceivedPacket received;
    while (link.receive(received)) {
      if (received.length != sizeof(tether::Telemetry)) continue;
      const auto &t = received.payload.telemetry;
      if (!tether::valid(t, received.length)) continue;
      if (print_status || (t.header.sequence % 25u) == 0u) {
        printf("node=%u seq=%u cmd=%u flags=0x%04x q=%.3f dq=%.3f tau=%.2f "
               "temp=%u/%uC motor_status=0x%x rssi=%d age=%luus can=%lu/%lu fail=%lu buserr=%lu\n",
               t.header.source_node, t.header.sequence, t.last_command_sequence,
               t.node_flags, t.position_rad, t.velocity_rad_s, t.torque_nm,
               t.mos_temperature_c, t.rotor_temperature_c, t.motor_error,
               t.command_rssi_dbm, static_cast<unsigned long>(t.command_age_us),
               static_cast<unsigned long>(t.can_rx_frames),
               static_cast<unsigned long>(t.can_tx_frames),
               static_cast<unsigned long>(t.can_tx_failures),
               static_cast<unsigned long>(t.can_bus_errors));
      }
    }
    print_status = false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

#else

void motorNodeRun() {
  tether::EspNowLink link;
  ESP_ERROR_CHECK(link.begin(kEspNowChannel));
  damiao::Motor motor(TETHER_MOTOR_SLAVE_ID, TETHER_MOTOR_MASTER_ID);
  ESP_ERROR_CHECK(motor.begin(kCanTx, kCanRx, kCanBitrate));

  bool estop = true;  // A reboot always returns inert.
  bool enabled = false;
  bool watchdog = true;
  bool limits_valid = false;
  uint16_t last_sequence = 0;
  bool have_command = false;
  uint32_t last_command_us = 0;
  uint32_t commands_received = 0;
  uint32_t command_gaps = 0;
  int8_t command_rssi = 0;
  uint8_t previous_action_flags = 0;
  tether::Command command = {};
  uint16_t telemetry_sequence = 0;
  uint32_t next_telemetry = nowMs();
  uint32_t next_probe = nowMs();
  uint32_t next_control = nowMs();

  ESP_LOGI("node", "node=%u CAN TX=GPIO%d RX=GPIO%d @ %lu bit/s; estop latched",
           TETHER_NODE_ID, kCanTx, kCanRx, static_cast<unsigned long>(kCanBitrate));

  for (;;) {
    motor.poll();
    motor.serviceBusRecovery();
    const uint32_t now_ms = nowMs();
    const uint32_t now_us = nowUs();

    if (!enabled && static_cast<int32_t>(now_ms - next_probe) >= 0) {
      (void)motor.refreshStatus();
      motor.poll();
      if (motor.seenRecently() && !limits_valid) limits_valid = motor.syncLimitsFromMotor();
      next_probe += 200;
    }

    tether::ReceivedPacket received;
    while (link.receive(received)) {
      if (received.length != sizeof(tether::Command)) continue;
      const auto &candidate = received.payload.command;
      if (!tether::valid(candidate, received.length) ||
          candidate.header.target_node != TETHER_NODE_ID) continue;
      if (have_command) {
        const int16_t delta = static_cast<int16_t>(candidate.header.sequence - last_sequence);
        if (delta <= 0) continue;
        if (delta > 1) command_gaps += static_cast<uint16_t>(delta - 1);
      }
      command = candidate;
      last_sequence = candidate.header.sequence;
      have_command = true;
      last_command_us = nowUs();
      command_rssi = received.rssi_dbm;
      ++commands_received;

      if (command.flags & tether::kEstop) estop = true;
      const uint8_t actions = command.flags &
          (tether::kResetEstop | tether::kZeroPosition);
      if ((actions & tether::kResetEstop) &&
          !(previous_action_flags & tether::kResetEstop) &&
          !(command.flags & tether::kEnable))
        estop = false;
      if ((actions & tether::kZeroPosition) &&
          !(previous_action_flags & tether::kZeroPosition) && !enabled && !estop)
        (void)motor.setZeroPosition();
      previous_action_flags = actions;
    }

    watchdog = !have_command || (now_us - last_command_us) > kWatchdogMs * 1000u;
    const damiao::BusHealth bus = motor.health();
    const bool bus_off = bus.state == TWAI_ERROR_BUS_OFF;
    const bool motor_fault = motor.feedback().error >= 0x8;
    if (bus_off || motor_fault || (enabled && !motor.seenRecently())) estop = true;

    const bool wants_enable = have_command && (command.flags & tether::kEnable) &&
                              command.mode != static_cast<uint8_t>(tether::ControlMode::kIdle);
    if (estop || watchdog || !wants_enable) {
      if (enabled) { (void)motor.disable(); enabled = false; }
    } else if (!enabled) {
      const uint8_t damiao_mode =
          command.mode == static_cast<uint8_t>(tether::ControlMode::kMit) ? 1 : 2;
      if (limits_valid && motor.seenRecently() && motor.setControlMode(damiao_mode)) {
        vTaskDelay(pdMS_TO_TICKS(20));
        enabled = motor.enable();
      }
    }

    if (enabled && !estop && !watchdog &&
        static_cast<int32_t>(now_ms - next_control) >= 0) {
      if (command.mode == static_cast<uint8_t>(tether::ControlMode::kPositionVelocity)) {
        (void)motor.commandPositionVelocity(command.position_rad, command.velocity_rad_s);
      } else if (command.mode == static_cast<uint8_t>(tether::ControlMode::kMit)) {
        (void)motor.commandMit(command.kp, command.kd, command.position_rad,
                               command.velocity_rad_s, command.torque_nm);
      }
      next_control = now_ms + 10;
    }

    if (static_cast<int32_t>(now_ms - next_telemetry) >= 0) {
      const auto &feedback = motor.feedback();
      const auto &limits = motor.limits();
      const damiao::BusHealth current_bus = motor.health();
      tether::Telemetry telemetry = {};
      telemetry.header.source_node = TETHER_NODE_ID;
      telemetry.header.target_node = 0;
      telemetry.header.sequence = ++telemetry_sequence;
      telemetry.header.sent_us = now_us;
      telemetry.last_command_sequence = last_sequence;
      telemetry.node_flags = (enabled ? tether::kNodeEnabled : 0) |
                             (watchdog ? tether::kNodeWatchdog : 0) |
                             (estop ? tether::kNodeEstop : 0) |
                             (current_bus.state == TWAI_ERROR_ACTIVE ? tether::kNodeCanActive : 0) |
                             (motor.seenRecently() ? tether::kNodeMotorSeen : 0) |
                             (limits_valid ? tether::kNodeLimitsValid : 0) |
                             (motor_fault ? tether::kNodeMotorFault : 0) |
                             (bus_off ? tether::kNodeBusOff : 0);
      telemetry.command_age_us = have_command ? now_us - last_command_us : UINT32_MAX;
      telemetry.uptime_ms = now_ms;
      telemetry.position_rad = feedback.position_rad;
      telemetry.velocity_rad_s = feedback.velocity_rad_s;
      telemetry.torque_nm = feedback.torque_nm;
      telemetry.position_limit_rad = limits.position_rad;
      telemetry.velocity_limit_rad_s = limits.velocity_rad_s;
      telemetry.torque_limit_nm = limits.torque_nm;
      telemetry.motor_error = feedback.error;
      telemetry.mos_temperature_c = feedback.mos_temperature_c;
      telemetry.rotor_temperature_c = feedback.rotor_temperature_c;
      telemetry.command_rssi_dbm = command_rssi;
      telemetry.can_tx_error_count = current_bus.tx_error_count;
      telemetry.can_rx_error_count = current_bus.rx_error_count;
      telemetry.can_bus_errors = current_bus.bus_errors;
      telemetry.can_rx_frames = current_bus.rx_frames;
      telemetry.can_tx_frames = current_bus.tx_frames;
      telemetry.can_tx_failures = current_bus.tx_failures;
      telemetry.command_received = commands_received;
      telemetry.command_gaps = command_gaps;
      telemetry.radio_rx_drops = link.receiveDrops();
      tether::seal(telemetry);
      (void)link.send(telemetry);
      next_telemetry += kTelemetryPeriodMs;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
#endif
}  // namespace

extern "C" void app_main() {
#if TETHER_ROLE == 0
  coordinatorRun();
#else
  static_assert(TETHER_NODE_ID >= 1 && TETHER_NODE_ID <= 3,
                "TETHER_NODE_ID must be 1..3");
  motorNodeRun();
#endif
}
