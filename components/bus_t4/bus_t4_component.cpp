#include "bus_t4_component.h"
#include "bus_t4.h"

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

#include "t4_packet.h"

#include <esp_rom_sys.h>

#include <algorithm>

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4";

// Upstream release this fork tracks, plus the branch, so the log says what is running
static const char *VERSION = "1.2.2+gate";

// Break signal baud rate: sending 0x00 at 9200 baud produces a ~1ms low pulse
// on a 19200 baud bus. Reference: pruwait/Nice_BusT4 nice-bust4.h
static constexpr uint32_t T4_BAUD_BREAK = 9200;

// Max message bytes per frame: T4Packet::data[63] less the 7-byte header and 1-byte CRC2.
static constexpr size_t T4_MAX_MESSAGE = 55;

// A request and its reply are separated by up to ~55ms on a shared bus, so treat the
// bus as free only after it has been quiet for longer than that.
static constexpr uint32_t BUS_IDLE_MS = 100;

// Bounded so a talkative bus cannot starve the transmit path
static constexpr uint8_t RX_DRAIN_LIMIT = 64;

// Minimum gap between two of our own transmissions
static constexpr TickType_t TX_MIN_INTERVAL = pdMS_TO_TICKS(100);

// Backoff for repeating discovery: a control unit that is busy at boot, or that is
// powered up later, is still found without flooding the bus in the meantime.
static constexpr uint32_t DISCOVERY_RETRY_MS[4] = {5000, 10000, 20000, 30000};

void BusT4Component::setup() {
  rxQueue_ = xQueueCreate(32, sizeof(T4Packet));
  if (rxQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    this->mark_failed();
    return;
  }

  txQueue_ = xQueueCreate(32, sizeof(T4Frame));
  if (txQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create TX queue");
    vQueueDelete(rxQueue_);
    rxQueue_ = nullptr;
    this->mark_failed();
    return;
  }

  if (xTaskCreate(busTaskThunk, "bus_t4", 8192, this, 10, &busTask_) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create bus task");
    this->mark_failed();
    return;
  }

  // Cache UART port number for direct baud rate changes during break signal.
  auto *idf_uart = static_cast<uart::IDFUARTComponent *>(parent_);
  uart_num_ = static_cast<uart_port_t>(idf_uart->get_hw_serial_number());

  this->discover_();
}

void BusT4Component::loop() {
  T4Packet packet;
  while (xQueueReceive(rxQueue_, &packet, 0))
    this->dispatch_packet_(packet);

  // Discovery used to be a single burst in setup(): a control unit that did not
  // answer in those three seconds left every device on the bus dead until the next
  // reboot. Keep asking instead; the reply arrives through dispatch_packet_.
  if (!discovered_) {
    const uint32_t interval = DISCOVERY_RETRY_MS[discovery_rounds_ < 3 ? discovery_rounds_ : 3];
    if (millis() - last_discovery_ms_ >= interval) {
      last_discovery_ms_ = millis();
      if (discovery_rounds_ < 3)
        discovery_rounds_++;
      ESP_LOGD(TAG, "Control unit still unknown, asking again");
      this->send_who_();
    }
  }

  if (bus_errors_sensor_ != nullptr && rx_errors_ != last_bus_errors_) {
    last_bus_errors_ = rx_errors_;
    bus_errors_sensor_->publish_state(rx_errors_);
  }
  if (bus_timeouts_sensor_ != nullptr && req_timeout_ != last_bus_timeouts_) {
    last_bus_timeouts_ = req_timeout_;
    bus_timeouts_sensor_->publish_state(req_timeout_);
  }
}

void BusT4Component::dispatch_packet_(const T4Packet &packet) {
  if (packet.header.from == address_) {
    ESP_LOGV(TAG, "Ignoring TX echo");
    return;
  }
  // A WHO reply can land here at any time, not just during the boot burst
  if (packet.header.protocol == DMP && packet.message.command == INF_WHO)
    this->handle_who_(packet);

  // Route by the message's target role: FOR_ALL fans out to everyone, otherwise
  // only devices that registered for that role (FOR_CU entities vs the OXI).
  auto role = static_cast<T4Target>(packet.message.device);
  for (auto *device : devices_)
    if (role == FOR_ALL || device->role() == role)
      device->on_packet(packet);
}

void BusT4Component::set_controller_address(T4Source addr) {
  for (auto *device : devices_)
    device->on_controller_resolved(addr);
}

std::string BusT4Component::fetch_string(T4Source to, uint8_t info_cmd) {
  uint8_t msg[5] = {FOR_ALL, info_cmd, REQ_GET, 0x00, 0x00};
  T4Packet reply;
  if (!this->dmp_request(to, msg, sizeof(msg), &reply, 500))
    return "";
  if (reply.size < 14)  // 7 header + 5 msg header + >=1 char + CRC
    return "";
  size_t len = reply.size - 13;  // data[12]..end, minus trailing CRC
  std::string s(reinterpret_cast<const char *>(&reply.data[12]), len);
  size_t nul = s.find('\0');
  if (nul != std::string::npos)
    s.resize(nul);

  // These strings go straight into the API, where a byte that is not valid UTF-8
  // breaks the protobuf stream: the client drops the connection, reconnects, reads
  // the same reply and loops. Controllers answer in ASCII, so anything outside
  // printable ASCII is padding (the RBA4R10 pads with 0xFF) and is dropped.
  std::string clean;
  clean.reserve(s.size());
  for (char ch : s) {
    const auto byte = static_cast<uint8_t>(ch);
    if (byte >= 0x20 && byte < 0x7F)
      clean.push_back(ch);
  }
  if (clean.size() != s.size())
    ESP_LOGD(TAG, "Dropped %u non-printable byte(s) from the reply to 0x%02X",
             static_cast<unsigned>(s.size() - clean.size()), info_cmd);
  return clean;
}

void BusT4Component::register_device(BusT4Device *device) {
  devices_.push_back(device);
  // Devices set up after this component, so one registering late still needs
  // whatever discovery has already resolved.
  if (discovered_)
    device->on_controller_resolved(controller_address_);
}

bool BusT4Component::send_who_() {
  T4Source broadcast{0xFF, 0xFF};
  uint8_t who[5] = {FOR_ALL, INF_WHO, REQ_GET, 0x00, 0x00};
  T4Packet req(broadcast, address_, DMP, who, sizeof(who));
  return this->write(&req, pdMS_TO_TICKS(200));
}

void BusT4Component::handle_who_(const T4Packet &pkt) {
  // The receive buffer is reused, so bytes past `size` are leftovers from an earlier
  // frame: never read the device type out of a short or rejected reply.
  const uint8_t payload = t4_dmp_payload_len(pkt);
  if (pkt.message.dmp.status != ERR_NONE || payload < 1) {
    ESP_LOGD(TAG, "Ignoring WHO reply from 0x%02X.%02X: status 0x%02X, %u payload byte(s)",
             pkt.header.from.address, pkt.header.from.endpoint, pkt.message.dmp.status,
             static_cast<unsigned>(payload));
    return;
  }

  const uint8_t type = pkt.message.dmp.data[0];  // device type byte
  if (type == CONTROLLER) {
    if (discovered_) {
      if (controller_address_ != pkt.header.from)
        ESP_LOGW(TAG, "Ignoring second control unit at 0x%02X.%02X, already using 0x%02X.%02X",
                 pkt.header.from.address, pkt.header.from.endpoint,
                 controller_address_.address, controller_address_.endpoint);
      return;
    }
    controller_address_ = pkt.header.from;
    discovered_ = true;
    ESP_LOGI(TAG, "Controller at 0x%02X.%02X%s", controller_address_.address,
             controller_address_.endpoint, has_oxi_ ? " (OXI present)" : "");
    this->set_controller_address(controller_address_);
  } else if (type == RADIO && !has_oxi_) {
    oxi_address_ = pkt.header.from;
    has_oxi_ = true;
    ESP_LOGI(TAG, "OXI at 0x%02X.%02X", oxi_address_.address, oxi_address_.endpoint);
  }
}

void BusT4Component::discover_() {
  // Short blocking burst at boot: devices set up right after this, so having the
  // address already resolved saves each of them a failed first read.
  TickType_t give_up = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
  while (!discovered_ && xTaskGetTickCount() < give_up) {
    if (!this->send_who_()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    TickType_t window = xTaskGetTickCount() + pdMS_TO_TICKS(400);
    while (!discovered_ && xTaskGetTickCount() < window) {
      T4Packet pkt;
      if (!this->read(&pkt, pdMS_TO_TICKS(20)))
        continue;
      if (pkt.header.from == address_)
        continue;
      if (pkt.header.protocol == DMP && pkt.message.command == INF_WHO)
        this->handle_who_(pkt);
      else
        this->dispatch_packet_(pkt);
    }
  }

  if (!discovered_) {
    ESP_LOGW(TAG, "No control unit answered in 3s; using default 0x%02X.%02X and retrying in the background",
             controller_address_.address, controller_address_.endpoint);
    this->set_controller_address(controller_address_);
  }
  last_discovery_ms_ = millis();
}

void BusT4Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BusT4:");
  ESP_LOGCONFIG(TAG, "  Version: %s", VERSION);
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", address_.address, address_.endpoint);
  if (discovered_) {
    ESP_LOGCONFIG(TAG, "  Controller: 0x%02X.%02X", controller_address_.address,
                  controller_address_.endpoint);
    if (has_oxi_)
      ESP_LOGCONFIG(TAG, "  OXI: 0x%02X.%02X", oxi_address_.address, oxi_address_.endpoint);
  } else {
    ESP_LOGCONFIG(TAG, "  Controller: not discovered");
  }
}

void BusT4Component::busTask() {
  T4Packet packet;
  uint8_t expected_size = 0;
  // Protocol format: [BREAK] SYNC SIZE DATA[N] SIZE
  // The size byte is sent twice (start and end), no separate checksum byte.
  // Internal checksums are within the DATA portion.
  enum { WAIT_SYNC = 0, SIZE, DATA, TRAILING_SIZE } rx_state = WAIT_SYNC;
  uint32_t last_rx_ms = 0;
  TickType_t last_tx_time = 0;

  for (;;) {
    uint8_t byte;
    uint8_t drained = 0;
    while (drained < RX_DRAIN_LIMIT && parent_->available() && parent_->read_byte(&byte) == true) {
      drained++;
      last_rx_ms = millis();
      switch (rx_state) {
        case WAIT_SYNC:
          // Wait for SYNC byte (0x55), ignore break bytes (0x00) and others
          if (byte == T4_SYNC) {
            rx_state = SIZE;
          }
          break;

        case SIZE:
          if (byte > 0 && byte <= 60) {
            expected_size = byte;
            packet.size = 0;
            rx_state = DATA;
          } else {
            // Invalid size, go back to waiting for sync
            rx_state = WAIT_SYNC;
          }
          break;

        case DATA:
          packet.data[packet.size++] = byte;
          if (packet.size == expected_size)
            rx_state = TRAILING_SIZE;
          break;

        case TRAILING_SIZE:
          // Verify trailing size matches
          if (byte == expected_size) {
            // Verify header checksum (CRC1): XOR of bytes 0-5 should equal byte 6
            uint8_t header_check = packet.checksum(0, 6);
            if (header_check != packet.data[6]) {
              ESP_LOGW(TAG, "Header checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                       header_check, packet.data[6],
                       format_hex_pretty(packet.data, packet.size).c_str());
              rx_errors_++;
              rx_state = WAIT_SYNC;
              break;
            }

            // Verify payload checksum (CRC2): XOR of bytes 7 to size-2 should equal byte size-1
            if (packet.size >= 9) {  // Minimum: 7 header + 1 payload + 1 CRC2
              uint8_t payload_check = packet.data[7];
              for (size_t i = 8; i < packet.size - 1; i++) {
                payload_check ^= packet.data[i];
              }
              if (payload_check != packet.data[packet.size - 1]) {
                ESP_LOGW(TAG, "Payload checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                         payload_check, packet.data[packet.size - 1],
                         format_hex_pretty(packet.data, packet.size).c_str());
                rx_errors_++;
                rx_state = WAIT_SYNC;
                break;
              }
            }

            // Both checksums valid - accept packet
            ESP_LOGD(TAG, "Received packet: %s (%d bytes)",
                     format_hex_pretty(packet.data, packet.size).c_str(), packet.size);
            if (!xQueueSend(rxQueue_, &packet, pdMS_TO_TICKS(100))) {
              ESP_LOGW(TAG, "RX queue full, dropping packet");
              rx_errors_++;
            }
          } else {
            ESP_LOGW(TAG, "Trailing size mismatch: expected 0x%02X, got 0x%02X", expected_size, byte);
            rx_errors_++;
          }
          rx_state = WAIT_SYNC;
          break;
      }
    }

    // Transmit between frames only, and only once the bus has gone quiet
    if (rx_state == WAIT_SYNC && millis() - last_rx_ms >= BUS_IDLE_MS) {
      T4Frame frame;
      if (xQueueReceive(txQueue_, &frame, 0)) {
        TickType_t elapsed = xTaskGetTickCount() - last_tx_time;
        if (elapsed < TX_MIN_INTERVAL)
          vTaskDelay(TX_MIN_INTERVAL - elapsed);

        if (frame.raw)
          ESP_LOGD(TAG, "Sending raw: %s", format_hex_pretty(frame.data, frame.size).c_str());
        else
          ESP_LOGD(TAG, "Sending packet: %s", format_hex_pretty(frame.data + 2, frame.size - 3).c_str());
        send_break();
        parent_->write_array(frame.data, frame.size);
        parent_->flush();

        last_tx_time = xTaskGetTickCount();
      }
    }

    vTaskDelay(1);
  }

  busTask_ = nullptr;
  vTaskDelete(nullptr);
}

bool BusT4Component::queue_frame(const T4Frame &frame, TickType_t xTicksToWait) {
  if (txQueue_ == nullptr)
    return false;
  return xQueueSend(txQueue_, &frame, xTicksToWait);
}

bool BusT4Component::write(T4Packet *packet, TickType_t xTicksToWait) {
  if (packet == nullptr || packet->size == 0 || packet->size > sizeof(T4Packet::data))
    return false;

  T4Frame frame;
  frame.data[0] = T4_SYNC;
  frame.data[1] = packet->size;
  std::copy_n(packet->data, packet->size, frame.data + 2);
  frame.data[packet->size + 2] = packet->size;
  frame.size = static_cast<uint8_t>(packet->size + 3);
  return queue_frame(frame, xTicksToWait);
}

std::vector<uint8_t> BusT4Component::parse_hex_(const std::string &s) {
  // Accepts separators ("55.0D", "55 0D") and bare hex ("550D"); empty result
  // signals invalid input (odd digit count or no hex) to callers.
  std::vector<uint8_t> bytes;
  int hi = -1;
  for (char ch : s) {
    uint8_t nib = parse_hex_char(ch);
    if (nib == INVALID_HEX_CHAR)
      continue;  // skip separators and other non-hex characters
    if (hi < 0) {
      hi = nib;
    } else {
      bytes.push_back(static_cast<uint8_t>((hi << 4) | nib));
      hi = -1;
    }
  }
  if (hi >= 0)
    return {};  // dangling nibble — odd number of hex digits
  return bytes;
}

void BusT4Component::dep_send(T4Source to, const uint8_t *msg, size_t len) {
  if (len > T4_MAX_MESSAGE) {
    ESP_LOGW(TAG, "dep_send: message too long (%u > %u)", static_cast<unsigned>(len), T4_MAX_MESSAGE);
    return;
  }
  T4Packet pkt(to, address_, DEP, msg, static_cast<uint8_t>(len));
  this->write(&pkt, 0);  // fire-and-forget: DEP is execute-only
}

void BusT4Component::dmp_send(T4Source to, const uint8_t *msg, size_t len) {
  if (len > T4_MAX_MESSAGE) {
    ESP_LOGW(TAG, "dmp_send: message too long (%u > %u)", static_cast<unsigned>(len), T4_MAX_MESSAGE);
    return;
  }
  T4Packet pkt(to, address_, DMP, msg, static_cast<uint8_t>(len));
  this->write(&pkt, 0);  // fire-and-forget: reply (if any) handled async in on_packet
}

bool BusT4Component::dmp_request(T4Source to, const uint8_t *msg, size_t len, T4Packet *reply,
                                 uint32_t timeout_ms) {
  if (len > T4_MAX_MESSAGE) {
    ESP_LOGW(TAG, "dmp_request: message too long (%u > %u)", static_cast<unsigned>(len), T4_MAX_MESSAGE);
    return false;
  }
  T4Packet req(to, address_, DMP, msg, static_cast<uint8_t>(len));
  if (!this->write(&req, pdMS_TO_TICKS(timeout_ms)))
    return false;

  uint8_t expected_cmd = req.message.command;
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() < deadline) {
    T4Packet pkt;
    if (!this->read(&pkt, pdMS_TO_TICKS(10)))
      continue;
    if (pkt.header.from == address_)  // skip TX echo
      continue;
    // Match source too: a FOR_ALL request is answered by both CU and OXI.
    if (pkt.header.protocol == DMP && pkt.message.command == expected_cmd && pkt.header.from == to) {
      *reply = pkt;
      return true;
    }
    this->dispatch_packet_(pkt);  // not ours — dispatch normally
  }
  this->req_timeout_++;
  return false;
}

std::string BusT4Component::debug_request(const std::string &message_hex, uint32_t timeout_ms) {
  std::vector<uint8_t> msg = parse_hex_(message_hex);
  if (msg.empty()) {
    ESP_LOGW(TAG, "[debug] invalid hex message: '%s'", message_hex.c_str());
    return "";
  }
  if (msg.size() > T4_MAX_MESSAGE) {
    ESP_LOGW(TAG, "[debug] message too long: %u bytes (max %u)", static_cast<unsigned>(msg.size()),
             T4_MAX_MESSAGE);
    return "";
  }

  T4Source target{0x00, 0x03};  // motor control unit
  ESP_LOGI(TAG, "[debug] TX dmp: %s", format_hex_pretty(msg).c_str());

  // dmp_request blocks until the matching reply arrives (short stall) or timeout.
  T4Packet rsp;
  if (this->dmp_request(target, msg.data(), msg.size(), &rsp, timeout_ms)) {
    std::string reply = format_hex_pretty(rsp.data, rsp.size);
    ESP_LOGI(TAG, "[debug] RX: %s", reply.c_str());
    return reply;
  }
  ESP_LOGI(TAG, "[debug] no reply within %ums", static_cast<unsigned>(timeout_ms));
  return "no reply";
}

std::string BusT4Component::debug_request_raw(const std::string &frame_hex, uint32_t timeout_ms) {
  std::vector<uint8_t> bytes = parse_hex_(frame_hex);
  if (bytes.empty()) {
    ESP_LOGW(TAG, "[debug] invalid hex frame: '%s'", frame_hex.c_str());
    return "";
  }
  ESP_LOGI(TAG, "[debug] TX raw: %s", format_hex_pretty(bytes).c_str());
  if (bytes.size() > sizeof(T4Frame::data)) {
    ESP_LOGW(TAG, "[debug] raw frame of %u bytes rejected", static_cast<unsigned>(bytes.size()));
    return "";
  }
  // Queued like everything else: nothing writes to the UART outside the bus task
  T4Frame raw_frame;
  raw_frame.raw = true;
  raw_frame.size = static_cast<uint8_t>(bytes.size());
  std::copy_n(bytes.data(), bytes.size(), raw_frame.data);
  queue_frame(raw_frame, 0);

  // Self-contained reply capture: byte-exact frames can't go through request()
  // (which re-frames), so wait here for the first non-echo packet, then return it.
  // Returns as soon as the reply lands (short stall), re-dispatching it so normal
  // device handling keeps working.
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
  while (xTaskGetTickCount() < deadline) {
    T4Packet pkt;
    if (!this->read(&pkt, pdMS_TO_TICKS(10)))
      continue;
    if (pkt.header.from == address_)  // skip our own TX echo
      continue;
    std::string reply = format_hex_pretty(pkt.data, pkt.size);
    ESP_LOGI(TAG, "[debug] RX: %s", reply.c_str());
    this->dispatch_packet_(pkt);  // keep normal device handling alive
    return reply;                 // first reply is the answer — stop waiting
  }
  ESP_LOGI(TAG, "[debug] no reply within %ums", static_cast<unsigned>(timeout_ms));
  return "no reply";
}

void BusT4Component::send_break() {
  // BusT4 protocol requires a break signal (~1ms low pulse) before each packet.
  // Achieved by sending 0x00 at a lower baud rate (9200 instead of 19200).
  // Reference: pruwait/Nice_BusT4 send_array_cmd() implementation.
  if (uart_num_ < UART_NUM_MAX) {
    uint32_t work_baud = parent_->get_baud_rate();
    uart_set_baudrate(uart_num_, T4_BAUD_BREAK);
    parent_->write_byte(T4_BREAK);
    parent_->flush();
    esp_rom_delay_us(100);
    uart_set_baudrate(uart_num_, work_baud);
  } else {
    // UART port not cached — fall back to simple break byte
    parent_->write_byte(T4_BREAK);
    parent_->flush();
  }
}

} // namespace esphome::bus_t4
