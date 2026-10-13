#pragma once

// Compile and upload id 0 first. Then change both values for each remote node.
// Arduino IDE does not have per-upload build profiles, so these are the only
// two role settings that should be edited between uploads.
#define TETHER_ROLE_MASTER 0
#define TETHER_ROLE_NODE 1

#ifndef TETHER_ROLE
#define TETHER_ROLE TETHER_ROLE_MASTER // MASTER for id 0; NODE for ids 1 and 2
#endif
#ifndef TETHER_NODE_ID
#define TETHER_NODE_ID 0               // master=0, left=1, right=2
#endif

// Board-specific CAN pins. The same firmware supports the S3 bench and the
// eventual XIAO ESP32-C6 without changing the packet protocol.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define TETHER_CAN_TX_GPIO 17
#define TETHER_CAN_RX_GPIO 18
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#define TETHER_CAN_TX_GPIO 20  // XIAO D9
#define TETHER_CAN_RX_GPIO 18  // XIAO D10
#else
#error "Select an ESP32-S3 or ESP32-C6 board"
#endif
#define TETHER_CAN_BITRATE 1000000

// Channel 1 is verified visible on the S3 bench. Every master/node build must
// use the same channel because SoftAP and ESP-NOW share one 2.4 GHz radio.
#define TETHER_ESPNOW_CHANNEL 1
#define TETHER_MASTER_AP_SSID "tether-control"
#define TETHER_MASTER_AP_PASSWORD "tether-bench"  // Change before field use.
#define TETHER_CONTROL_UDP_PORT 4210
#define TETHER_LAPTOP_WATCHDOG_MS 1000
// Each module owns one local CAN bus with a small motor (slot 0) and a big
// motor (slot 1). CAN IDs are local to that bus and independent of Tether IDs.
// A module will advertise over ESP-NOW but refuse motor control while its four
// IDs are unconfigured. Fill IDs only from a physical scan.
#if TETHER_NODE_ID == 0
#define TETHER_MOTORS_CONFIGURED 1
#define TETHER_SMALL_MOTOR_SLAVE_ID 0x07
#define TETHER_SMALL_MOTOR_FEEDBACK_ID 0x17
#define TETHER_BIG_MOTOR_SLAVE_ID 0x04
#define TETHER_BIG_MOTOR_FEEDBACK_ID 0x14
#elif TETHER_NODE_ID == 1
#define TETHER_MOTORS_CONFIGURED 0
#define TETHER_SMALL_MOTOR_SLAVE_ID 0x000
#define TETHER_SMALL_MOTOR_FEEDBACK_ID 0x000
#define TETHER_BIG_MOTOR_SLAVE_ID 0x000
#define TETHER_BIG_MOTOR_FEEDBACK_ID 0x000
#elif TETHER_NODE_ID == 2
#define TETHER_MOTORS_CONFIGURED 0
#define TETHER_SMALL_MOTOR_SLAVE_ID 0x000
#define TETHER_SMALL_MOTOR_FEEDBACK_ID 0x000
#define TETHER_BIG_MOTOR_SLAVE_ID 0x000
#define TETHER_BIG_MOTOR_FEEDBACK_ID 0x000
#else
#error "TETHER_NODE_ID must be 0, 1, or 2"
#endif

#define TETHER_COMMAND_PERIOD_MS 20
#define TETHER_TELEMETRY_PERIOD_MS 40
#define TETHER_HELLO_PERIOD_MS 1000
#define TETHER_PEER_OFFLINE_MS 2500
#define TETHER_COMMAND_WATCHDOG_US 60000
