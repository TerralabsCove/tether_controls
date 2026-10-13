#include "DamiaoMotor.h"

#include <esp_twai_onchip.h>

static float bound(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

bool DamiaoMotor::begin(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate) {
  queue_ = xQueueCreate(64, sizeof(Rx)); if (!queue_) return false;
  twai_onchip_node_config_t c = {};
  c.io_cfg.tx = tx; c.io_cfg.rx = rx; c.io_cfg.quanta_clk_out = GPIO_NUM_NC;
  c.io_cfg.bus_off_indicator = GPIO_NUM_NC; c.bit_timing.bitrate = bitrate;
  c.fail_retry_cnt = 3; c.tx_queue_depth = 16; c.flags.no_receive_rtr = true;
  if (twai_new_node_onchip(&c, &node_) != ESP_OK) return false;
  twai_event_callbacks_t callbacks = {}; callbacks.on_rx_done = rxCallback;
  return twai_node_register_event_callbacks(node_, &callbacks, this) == ESP_OK &&
         twai_node_enable(node_) == ESP_OK;
}
bool IRAM_ATTR DamiaoMotor::rxCallback(twai_node_handle_t h,
    const twai_rx_done_event_data_t *, void *ctx) {
  auto *self = static_cast<DamiaoMotor *>(ctx); Rx r = {};
  twai_frame_t f = {}; f.buffer = r.data; f.buffer_len = 8;
  if (twai_node_receive_from_isr(h, &f) != ESP_OK || f.header.ide || f.header.rtr) return false;
  r.id = f.header.id; r.length = twaifd_dlc2len(f.header.dlc); BaseType_t wake = pdFALSE;
  if (xQueueSendFromISR(self->queue_, &r, &wake) != pdTRUE) {
    self->drops_ = self->drops_ + 1u;
  }
  return wake == pdTRUE;
}
bool DamiaoMotor::send(uint32_t id, const uint8_t bytes[8]) {
  uint8_t data[8]; memcpy(data, bytes, 8); twai_frame_t f = {};
  f.header.id = id; f.header.dlc = 8; f.buffer = data; f.buffer_len = 8;
  if (twai_node_transmit(node_, &f, 20) != ESP_OK ||
      twai_node_transmit_wait_all_done(node_, 20) != ESP_OK) { tx_fail_++; return false; }
  tx_++; return true;
}
bool DamiaoMotor::control(uint8_t b) {
  const uint8_t d[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,b}; return send(slave_, d);
}
bool DamiaoMotor::enable() { return control(0xfc); }
bool DamiaoMotor::disable() { return control(0xfd); }
bool DamiaoMotor::zero() { return control(0xfe); }
bool DamiaoMotor::refresh() { return control(0xcc); }
bool DamiaoMotor::setControlMode(uint8_t mode) {
  uint8_t d[8] = {uint8_t(slave_),uint8_t(slave_>>8),0x55,10,mode,0,0,0};
  return send(0x7ff,d);
}
bool DamiaoMotor::posVel(float q, float dq) {
  q = bound(q, -limits_.q, limits_.q); dq = bound(fabsf(dq), 0, limits_.dq);
  uint8_t d[8]; memcpy(d, &q, 4); memcpy(d + 4, &dq, 4); return send(slave_ + 0x100, d);
}
bool DamiaoMotor::mit(float q, float dq, float tau, float kp, float kd) {
  uint16_t p=pack(q,-limits_.q,limits_.q,16), v=pack(dq,-limits_.dq,limits_.dq,12);
  uint16_t pg=pack(kp,0,500,12), dg=pack(kd,0,5,12), t=pack(tau,-limits_.tau,limits_.tau,12);
  uint8_t d[8] = {uint8_t(p>>8),uint8_t(p),uint8_t(v>>4),uint8_t((v&15)<<4|pg>>8),
    uint8_t(pg),uint8_t(dg>>4),uint8_t((dg&15)<<4|t>>8),uint8_t(t)};
  return send(slave_, d);
}
void DamiaoMotor::poll(size_t maximum) {
  Rx r; while (maximum-- && xQueueReceive(queue_, &r, 0) == pdTRUE) { rx_++; decode(r); }
}
bool DamiaoMotor::decode(const Rx &r) {
  if (r.length < 8 || r.data[2] == 0x33 || r.data[2] == 0x55) return false;
  if (r.id != master_ && r.id != slave_ && r.id != 0) return false;
  uint16_t p=uint16_t(r.data[1])<<8|r.data[2], v=uint16_t(r.data[3])<<4|r.data[4]>>4;
  uint16_t t=uint16_t(r.data[4]&15)<<8|r.data[5];
  feedback_.q=unpack(p,-limits_.q,limits_.q,16); feedback_.dq=unpack(v,-limits_.dq,limits_.dq,12);
  feedback_.tau=unpack(t,-limits_.tau,limits_.tau,12); feedback_.error=r.data[0]>>4;
  feedback_.mos_c=r.data[6]; feedback_.rotor_c=r.data[7]; feedback_.frames++; feedback_.last_ms=millis();
  return true;
}
bool DamiaoMotor::readRegister(uint8_t id, float &value) {
  uint8_t d[8] = {uint8_t(slave_),uint8_t(slave_>>8),0x33,id,0,0,0,0};
  if (!send(0x7ff,d)) return false; uint32_t deadline=millis()+100; Rx r;
  while (int32_t(millis()-deadline)<0) {
    if (xQueueReceive(queue_,&r,pdMS_TO_TICKS(5))!=pdTRUE) continue; rx_++;
    if (r.length==8 && r.data[2]==0x33 && r.data[3]==id &&
        (uint16_t(r.data[1])<<8|r.data[0])==slave_) {
      memcpy(&value,r.data+4,4); return isfinite(value)&&value>0;
    } decode(r);
  } return false;
}
bool DamiaoMotor::syncLimits() {
  float q,dq,tau; if(!readRegister(21,q)||!readRegister(22,dq)||!readRegister(23,tau)) return false;
  limits_.q=q; limits_.dq=dq; limits_.tau=tau; limits_.valid=true; return true;
}
bool DamiaoMotor::seen(uint32_t age) const { return feedback_.frames && millis()-feedback_.last_ms<=age; }
void DamiaoMotor::recover() { twai_node_status_t s; if(twai_node_get_info(node_,&s,nullptr)==ESP_OK&&s.state==TWAI_ERROR_BUS_OFF) twai_node_recover(node_); }
CanHealth DamiaoMotor::health() const {
  CanHealth h; twai_node_status_t s; twai_node_record_t r;
  if(twai_node_get_info(node_,&s,&r)==ESP_OK){h.state=s.state;h.tx_error=s.tx_error_count;h.rx_error=s.rx_error_count;h.bus_errors=r.bus_err_num;}
  h.rx_frames=rx_;h.tx_frames=tx_;h.tx_failures=tx_fail_;h.rx_drops=drops_;return h;
}
uint16_t DamiaoMotor::pack(float x,float lo,float hi,uint8_t bits){x=bound(x,lo,hi);return uint16_t((x-lo)*((1u<<bits)-1)/(hi-lo));}
float DamiaoMotor::unpack(uint16_t x,float lo,float hi,uint8_t bits){return float(x)*(hi-lo)/float((1u<<bits)-1)+lo;}
