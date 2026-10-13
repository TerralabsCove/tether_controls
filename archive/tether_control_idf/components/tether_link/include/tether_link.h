#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_now.h"
#include "tether_protocol.h"

namespace tether {

struct ReceivedPacket {
  uint8_t source_mac[6];
  int8_t rssi_dbm;
  size_t length;
  union {
    Command command;
    Telemetry telemetry;
  } payload;
};

class EspNowLink {
 public:
  esp_err_t begin(uint8_t channel = 13);
  bool receive(ReceivedPacket &packet, uint32_t timeout_ms = 0);
  esp_err_t send(const Command &packet);
  esp_err_t send(const Telemetry &packet);
  uint32_t receiveDrops() const { return receive_drops_; }

 private:
  static void receiveCallback(const esp_now_recv_info_t *info,
                              const uint8_t *data, int length);
  esp_err_t sendBytes(const void *data, size_t length);

  static EspNowLink *instance_;
  void *queue_ = nullptr;
  volatile uint32_t receive_drops_ = 0;
};

}  // namespace tether
