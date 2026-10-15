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
// C5 migration (leave disabled until XIAO ESP32-C5 hardware is tested):
// Replace the C6 branch above with this branch, or add it as another active
// #elif. Seeed maps XIAO C5 D9/D10 to GPIO9/GPIO10, unlike the C6.
// #elif defined(CONFIG_IDF_TARGET_ESP32C5)
// #define TETHER_CAN_TX_GPIO 9   // XIAO ESP32-C5 D9; verify with transceiver
// #define TETHER_CAN_RX_GPIO 10  // XIAO ESP32-C5 D10; verify with transceiver
#else
#error "Select an ESP32-S3 or ESP32-C6 board"
// C5 migration: change the message above to "Select an ESP32-S3, ESP32-C6, or ESP32-C5 board".
#endif
#define TETHER_CAN_BITRATE 1000000

// Channel 1 is verified visible on the S3 bench. Every master/node build must
// use the same channel because SoftAP and ESP-NOW share one 2.4 GHz radio.
#define TETHER_ESPNOW_CHANNEL 1
// C5 5 GHz migration (do not enable until every module is a C5):
// Replace channel 1 above with a locally permitted non-DFS channel. Channel 36
// is the initial bench-test candidate. SoftAP and every ESP-NOW peer must use
// exactly the same channel.
// #define TETHER_ESPNOW_CHANNEL 36
#define TETHER_MASTER_AP_SSID "tether-control"
#define TETHER_MASTER_AP_PASSWORD "tether-bench"  // Change before field use.
#define TETHER_CONTROL_UDP_PORT 4210
#define TETHER_LAPTOP_WATCHDOG_MS 1000
// Each module owns one local CAN bus. Master id 0 has one motor; remote ids 1
// and 2 have three. CAN IDs are local to that bus and independent of Tether
// IDs. A remote module advertises over ESP-NOW but refuses motor control while
// any of its IDs are unconfigured. Fill IDs only from a physical scan.
#if TETHER_NODE_ID == 0
#define TETHER_LOCAL_MOTOR_COUNT 1
#define TETHER_MOTORS_CONFIGURED 1
#define TETHER_MOTOR_0_SLAVE_ID 0x04
#define TETHER_MOTOR_0_FEEDBACK_ID 0x14
#define TETHER_MOTOR_1_SLAVE_ID 0x000
#define TETHER_MOTOR_1_FEEDBACK_ID 0x000
#define TETHER_MOTOR_2_SLAVE_ID 0x000
#define TETHER_MOTOR_2_FEEDBACK_ID 0x000
#elif TETHER_NODE_ID == 1
#define TETHER_LOCAL_MOTOR_COUNT 3
#define TETHER_MOTORS_CONFIGURED 0
#define TETHER_MOTOR_0_SLAVE_ID 0x02  // big
#define TETHER_MOTOR_0_FEEDBACK_ID 0x000
#define TETHER_MOTOR_1_SLAVE_ID 0x05  // closer to master
#define TETHER_MOTOR_1_FEEDBACK_ID 0x000
#define TETHER_MOTOR_2_SLAVE_ID 0x01  // farther/end
#define TETHER_MOTOR_2_FEEDBACK_ID 0x000
#elif TETHER_NODE_ID == 2
#define TETHER_LOCAL_MOTOR_COUNT 3
#define TETHER_MOTORS_CONFIGURED 0
#define TETHER_MOTOR_0_SLAVE_ID 0x06  // big
#define TETHER_MOTOR_0_FEEDBACK_ID 0x000
#define TETHER_MOTOR_1_SLAVE_ID 0x03  // closer to master
#define TETHER_MOTOR_1_FEEDBACK_ID 0x000
#define TETHER_MOTOR_2_SLAVE_ID 0x07  // farther/end
#define TETHER_MOTOR_2_FEEDBACK_ID 0x000
#else
#error "TETHER_NODE_ID must be 0, 1, or 2"
#endif

#define TETHER_COMMAND_PERIOD_MS 20
#define TETHER_TELEMETRY_PERIOD_MS 40
#define TETHER_HELLO_PERIOD_MS 1000
#define TETHER_PEER_OFFLINE_MS 2500
#define TETHER_COMMAND_WATCHDOG_US 60000
