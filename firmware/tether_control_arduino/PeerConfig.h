#pragma once

#include <Arduino.h>

struct PeerDefinition {
  uint8_t id;
  const char *name;
  uint8_t mac[6];
};

// kMasterMac is only the master identity check. kMasterApMac is the address
// every node MUST use as its ESP-NOW peer because the master runs AP-only.
// Do not put the master's station MAC in kMasterApMac. A zero MAC is treated
// as unconfigured so a partially filled table cannot enable a motor.
static constexpr uint8_t kMasterMac[6] =
    {0xB4, 0x3A, 0x45, 0x89, 0x77, 0x20};
static constexpr uint8_t kMasterApMac[6] =
    {0xB4, 0x3A, 0x45, 0x89, 0x77, 0x21};
static constexpr PeerDefinition kNodePeers[] = {
    {1, "left",  {0x10, 0xBD, 0xA3, 0x90, 0x82, 0xC8}},
    {2, "right", {0x10, 0xBD, 0xA3, 0x90, 0xEA, 0x44}},
};
static constexpr size_t kNodePeerCount = sizeof(kNodePeers) / sizeof(kNodePeers[0]);

/*
MAC probe records used for this configuration:

0
chip=XIAO_ESP32C6 Arduino=4.0.0 IDF=v6.1-803-g94639ab7251
station_mac=B4:3A:45:89:77:20
station_mac initializer={0xB4, 0x3A, 0x45, 0x89, 0x77, 0x20}
softap_mac=B4:3A:45:89:77:21
softap_mac initializer={0xB4, 0x3A, 0x45, 0x89, 0x77, 0x21}
master kMasterMac <- station_mac
master kMasterApMac <- softap_mac (REQUIRED ESP-NOW peer address)
node kNodePeers row <- station_mac

1
chip=XIAO_ESP32C6 Arduino=4.0.0 IDF=v6.1-803-g94639ab7251
station_mac=10:BD:A3:90:82:C8
station_mac initializer={0x10, 0xBD, 0xA3, 0x90, 0x82, 0xC8}
softap_mac=10:BD:A3:90:82:C9
softap_mac initializer={0x10, 0xBD, 0xA3, 0x90, 0x82, 0xC9}
master kMasterMac <- station_mac
master kMasterApMac <- softap_mac (REQUIRED ESP-NOW peer address)
node kNodePeers row <- station_mac

2
chip=XIAO_ESP32C6 Arduino=4.0.0 IDF=v6.1-803-g94639ab7251
station_mac=10:BD:A3:90:EA:44
station_mac initializer={0x10, 0xBD, 0xA3, 0x90, 0xEA, 0x44}
softap_mac=10:BD:A3:90:EA:45
softap_mac initializer={0x10, 0xBD, 0xA3, 0x90, 0xEA, 0x45}
master kMasterMac <- station_mac
master kMasterApMac <- softap_mac (REQUIRED ESP-NOW peer address)
node kNodePeers row <- station_mac
*/

inline bool macIsZero(const uint8_t mac[6]) {
  for (size_t i = 0; i < 6; i++) if (mac[i] != 0) return false;
  return true;
}

inline bool macEqual(const uint8_t lhs[6], const uint8_t rhs[6]) {
  return memcmp(lhs, rhs, 6) == 0;
}

inline const PeerDefinition *peerForId(uint8_t id) {
  for (size_t i = 0; i < kNodePeerCount; i++)
    if (kNodePeers[i].id == id) return &kNodePeers[i];
  return nullptr;
}
