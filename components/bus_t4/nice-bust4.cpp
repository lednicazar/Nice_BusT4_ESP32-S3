#include "nice-bust4.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  CoverTraits traits;
  traits.set_supports_position(true);
  traits.set_supports_stop(true);

  // tienes feedback real de posición/estado -> no "assumed"
  traits.set_is_assumed_state(false);
  traits.set_supports_tilt(false);

  return traits;
}

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;

  if (call.get_stop()) {
    send_cmd(STOP);
    return;
  }

  if (call.get_position().has_value()) {
    const float newpos = *call.get_position();

    // Si ya estamos ahí, no hagas nada
    if (newpos == this->position)
      return;

    if (newpos == COVER_OPEN) {
      if (current_operation != COVER_OPERATION_OPENING)
        send_cmd(OPEN);
      return;
    }

    if (newpos == COVER_CLOSED) {
      if (current_operation != COVER_OPERATION_CLOSING)
        send_cmd(CLOSE);
      return;
    }

    // Posición intermedia
    position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
    ESP_LOGI(TAG, "Required actuator position: %u", (unsigned) position_hook_value);

    if (position_hook_value > _pos_usl) {
      position_hook_type = STOP_UP;
      if (current_operation != COVER_OPERATION_OPENING)
        send_cmd(OPEN);
    } else {
      position_hook_type = STOP_DOWN;
      if (current_operation != COVER_OPERATION_CLOSING)
        send_cmd(CLOSE);
    }
  }
}

void NiceBusT4::setup() {
  this->current_operation = COVER_OPERATION_IDLE;
  this->last_published_op = COVER_OPERATION_IDLE;
  this->last_published_pos = this->position;  // normalmente 0 al inicio

  // si quieres publicar el estado inicial:
  // this->publish_state();
}

void NiceBusT4::loop() {
  if ((millis() - this->last_update_) > 10000) {  // every 10 seconds
    std::vector<uint8_t> unknown = {0x55, 0x55};

    if (!this->init_ok) {
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00));
    } else if (this->class_gate_ == 0x55) {
      init_device(this->addr_to[0], this->addr_to[1], 0x04);
    } else if (this->manufacturer_ == unknown) {
      init_device(this->addr_to[0], this->addr_to[1], 0x04);
    }

    this->last_update_ = millis();
  }

  // allow sending every 100 ms
  uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  }

  while (this->parent_->available() > 0) {
    uint8_t c = this->read();
    this->handle_char_(c);
    this->last_uart_byte_ = now;
  }

  if (this->ready_to_tx_ && !this->tx_buffer_.empty()) {
    this->send_array_cmd(this->tx_buffer_.front());
    this->tx_buffer_.pop();
    this->ready_to_tx_ = false;
  }

  // Polling of the current actuator position
  if (!is_robus) {
    now = millis();
    if (init_ok && (current_operation != COVER_OPERATION_IDLE) && (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
      last_position_time = now;
      request_position();
    }
  }
}

void NiceBusT4::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);
  if (!this->validate_message_()) {
    this->rx_message_.clear();
  }
}

bool NiceBusT4::validate_message_() {
  uint32_t at = this->rx_message_.size() - 1;
  uint8_t *data = &this->rx_message_[0];
  uint8_t new_byte = data[at];

  if (at == 0) return new_byte == 0x00;
  if (at == 1) return new_byte == START_CODE;
  if (at == 2) return true;
  if (at == 3) return true;
  if (at <= 8) return true;

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

  if (at == 9) {
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
      return false;
    }
  }

  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3);

  if (at < length) return true;

  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length - 1], crc2);
    return false;
  }

  if (data[length] != packet_size) {
    ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[length], packet_size);
    return false;
  }

  rx_message_.erase(rx_message_.begin());

  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG, "Packet received: %s", pretty_cmd.c_str());

  parse_status_packet(rx_message_);

  return false;
}

// ... aquí el resto de tu .cpp sigue igual, pero con CAMBIO IMPORTANTE:
// reemplazar TODOS los "%S" por "%s" en ESP_LOGx, porque usas c_str()

// Requesting the conditional current position of the actuator
void NiceBusT4::request_position(void) {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

// Updating the current position of the actuator
void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;

  const int32_t denom = (int32_t) _pos_opn - (int32_t) _pos_cls;
  if (denom == 0) {
    ESP_LOGW(TAG, "Invalid calibration (_pos_opn == _pos_cls). Skipping position update.");
    return;
  }

  position = (_pos_usl - _pos_cls) * 1.0f / denom;
  ESP_LOGI(TAG, "Conditional goal position: %u, position: %.3f", (unsigned) newpos, position);

  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;

  publish_state_if_changed();

  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) ||
      (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
    ESP_LOGI(TAG, "Desired position reached. Stopping the gate");
    send_cmd(STOP);
    position_hook_type = IGNORE;
  }
}

void NiceBusT4::publish_state_if_changed(void) {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;

  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

}  // namespace bus_t4
}  // namespace esphome
