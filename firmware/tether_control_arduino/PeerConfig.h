#pragma once

#include <Arduino.h>

struct PeerDefinition {
  uint8_t id;
  const char *name;
  uint8_t mac[6];
};

// Fill these from mac_probe/mac_probe.ino for the actual boards. The values
// below are intentionally zero and MUST be provisioned for the actual boards.
//
// kMasterMac is only the master identity check. kMasterApMac is the address
// every node MUST use as its ESP-NOW peer because the master runs AP-only.
// Do not put the master's station MAC in kMasterApMac. A zero MAC is treated
// as unconfigured so a partially filled table cannot enable a motor.
static constexpr uint8_t kMasterMac[6] = {0, 0, 0, 0, 0, 0};
static constexpr uint8_t kMasterApMac[6] = {0, 0, 0, 0, 0, 0};
static constexpr PeerDefinition kNodePeers[] = {
    {1, "left",  {0, 0, 0, 0, 0, 0}},
    {2, "right", {0, 0, 0, 0, 0, 0}},
};
static constexpr size_t kNodePeerCount = sizeof(kNodePeers) / sizeof(kNodePeers[0]);

inline bool macIsZero(const uint8_t mac[6]) {
  for (size_t i = 0; i < 6; ++i) if (mac[i] != 0) return false;
  return true;
}

inline bool macEqual(const uint8_t lhs[6], const uint8_t rhs[6]) {
  return memcmp(lhs, rhs, 6) == 0;
}

inline const PeerDefinition *peerForId(uint8_t id) {
  for (size_t i = 0; i < kNodePeerCount; ++i)
    if (kNodePeers[i].id == id) return &kNodePeers[i];
  return nullptr;
}
