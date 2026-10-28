#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <functional>
#include <string>
#include <vector>
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "t4_packet.h"

#include <driver/uart.h>
#include "esphome/components/uart/uart_component_esp_idf.h"

namespace esphome::bus_t4 {

// Forward declaration
class BusT4Device;

// A queued transmission, already framed. Packets and raw sends share one path so that
// nothing writes to the UART outside the bus task.
struct T4Frame {
  uint8_t size = 0;
  bool raw = false;
  uint8_t data[sizeof(T4Packet::data) + 3] = {};  // SYNC + size + payload + size
};

class BusT4Component final : public Component, public uart::UARTDevice {
 public:
  BusT4Component() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  bool read(T4Packet *packet, TickType_t xTicksToWait) {
    if (rxQueue_ == nullptr)
      return false;
    return xQueueReceive(rxQueue_, packet, xTicksToWait);
  }

  // Frame `packet` and queue it; the bus task is what puts it on the wire.
  bool write(T4Packet *packet, TickType_t xTicksToWait);

  // --- Send primitives (used by devices via parent_, and by the debug helpers) ---
  // Verb says whether a reply is expected: `send` fires and returns; `request` waits.
  void dep_send(T4Source to, const uint8_t *msg, size_t len);  // DEP (execute-only, no reply)
  void dmp_send(T4Source to, const uint8_t *msg, size_t len);  // DMP, async (reply via on_packet)
  // DMP request: frame `msg` to `to`, send, and block up to timeout_ms for the
  // matching reply (same command byte); non-matching packets are dispatched to
  // devices normally. Returns true and fills `reply`. Don't call from a hot loop —
  // it blocks; the cover's periodic polling uses async dmp_send instead.
  bool dmp_request(T4Source to, const uint8_t *msg, size_t len, T4Packet *reply, uint32_t timeout_ms = 500);

  // --- Interactive debugging (hex wrappers, callable from YAML lambdas / API actions) ---
  // DMP request to the control unit from message bytes; returns the reply as a hex
  // string (e.g. "00.81.00.03..."), or "no reply" on timeout / "" on bad input.
  // e.g. debug_request("04 D1 99 00 00") reads IO state.
  std::string debug_request(const std::string &message_hex, uint32_t timeout_ms = 500);
  // Send byte-exact frame bytes (no framing/CRCs added — you supply the whole frame).
  std::string debug_request_raw(const std::string &frame_hex, uint32_t timeout_ms = 500);

  void set_address(const uint16_t address) {
    address_.address = static_cast<uint8_t>(address >> 8);
    address_.endpoint = static_cast<uint8_t>(address & 0xFF);
  }

  T4Source get_address() const { return address_; }

  // Register a device to receive packet callbacks
  void register_device(BusT4Device *device);

  void set_bus_errors_sensor(sensor::Sensor *s) { bus_errors_sensor_ = s; }
  void set_bus_timeouts_sensor(sensor::Sensor *s) { bus_timeouts_sensor_ = s; }

  // Blocking single-shot identity read: GET `info_cmd` from `to`, return the
  // reply's string payload (empty on timeout). Used by device components at setup.
  std::string fetch_string(T4Source to, uint8_t info_cmd);

  // Propagate the discovered controller address to every registered device
  // (each ignores it if its address was pinned via config).
  void set_controller_address(T4Source addr);

  bool discovery_ready() const { return discovered_; }
  T4Source get_controller_address() const { return controller_address_; }
  T4Source get_oxi_address() const { return oxi_address_; }
  bool has_oxi() const { return has_oxi_; }

 private:
  void discover_();

  // Broadcast "who is on the bus"; the reply is handled wherever it lands
  bool send_who_();
  void handle_who_(const T4Packet &pkt);

  // Receive and transmit share one task, so a send can never land in the middle of
  // a frame another device is still transmitting.
  void busTask();
  static void busTaskThunk(void *self) { static_cast<BusT4Component *>(self)->busTask(); }

  bool queue_frame(const T4Frame &frame, TickType_t xTicksToWait);

  // Dispatch a received packet to all registered devices (skips TX echo).
  void dispatch_packet_(const T4Packet &packet);

  // Send a BusT4 break signal (~1ms low pulse) before each packet.
  // Temporarily lowers UART baud rate to produce the correct break duration.
  void send_break();

  // Parse a hex string ("04 D1 99" or "04D199") into bytes. Returns empty on
  // an odd digit count or no hex digits (callers treat empty as invalid input).
  static std::vector<uint8_t> parse_hex_(const std::string &s);

  T4Source address_;

  TaskHandle_t busTask_ = nullptr;

  QueueHandle_t rxQueue_ = nullptr;
  QueueHandle_t txQueue_ = nullptr;

  std::vector<BusT4Device *> devices_;

  bool discovered_{false};
  uint32_t last_discovery_ms_{0};   // when the last WHO went out
  uint8_t discovery_rounds_{0};     // retries so far, for the backoff
  T4Source controller_address_{0x00, 0x03};
  T4Source oxi_address_{0x00, 0x00};
  bool has_oxi_{false};

  // Cached UART port for direct baud rate register writes during break signal.
  uart_port_t uart_num_ = UART_NUM_MAX;

  sensor::Sensor *bus_errors_sensor_{nullptr};
  sensor::Sensor *bus_timeouts_sensor_{nullptr};

  // Bus diagnostics (cumulative since boot); written by the bus task/dmp_request.
  uint32_t rx_errors_{0};    // bad frame: header/payload CRC, framing, or rxQueue full (logged per type)
  uint32_t req_timeout_{0};  // dmp_request gave up waiting for a reply
  uint32_t last_bus_errors_{0xFFFFFFFF};
  uint32_t last_bus_timeouts_{0xFFFFFFFF};
};

} // namespace esphome::bus_t4
