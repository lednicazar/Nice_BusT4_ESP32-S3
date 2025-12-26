#include "nice-bust4.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(false);
  traits.set_supports_tilt(false);
  return traits;
}

/*
  command dumps overview

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c
*/

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);

  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);

      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);

      } else { // Arbitrary position
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Required actuator position: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {
  this->current_operation = COVER_OPERATION_IDLE;
  this->last_published_op = this->current_operation;
  this->last_published_pos = this->position;
}

void NiceBusT4::loop() {

  if ((millis() - this->last_update_) > 10000) {    // every 10 seconds
    // If the drive is not detected the first time, we will try again later.
    std::vector<uint8_t> unknown = {0x55, 0x55};
    if (this->init_ok == false) {
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //product request
    }

    else if (this->class_gate_ == 0x55) {
      init_device(this->addr_to[0], this->addr_to[1], 0x04);
    }
    else if (this->manufacturer_ == unknown)  {
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
    uint8_t c = this->read();                // read byte
    this->handle_char_(c);                   // send byte for processing
    this->last_uart_byte_ = now;
  }

  if (this->ready_to_tx_) {   // if it's okay to send
    if (!this->tx_buffer_.empty()) {  // if there's anything to send
      this->send_array_cmd(this->tx_buffer_.front()); // send the first team in line
      this->tx_buffer_.pop();
      this->ready_to_tx_ = false;
    }
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
  this->rx_message_.push_back(c); // throw a byte at the end of the received message
  if (!this->validate_message_()) {
    this->rx_message_.clear();
  }
}

bool NiceBusT4::validate_message_() {
  uint32_t at = this->rx_message_.size() - 1;
  uint8_t *data = &this->rx_message_[0];
  uint8_t new_byte = data[at];

  // Byte 0: HEADER1 (always 0x00)
  if (at == 0)
    return new_byte == 0x00;
  // Byte 1: HEADER2 (always 0x55)
  if (at == 1)
    return new_byte == START_CODE;

  // Byte 2: packet_size - number of bytes further + 1
  if (at == 2)
    return true;
  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3);

  // Byte 3: Series (row) to whom package
  if (at == 3)
    return true;

  if (at <= 8)
    return true;

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

  // Byte 9: crc1 = XOR (Byte 3 : Byte 8)
  if (at == 9)
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
      return false;
    }

  // waiting for all the package data to come in.
  if (at < length)
    return true;

  // calculate crc2
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2 ) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length - 1], crc2);
    return false;
  }

  // Byte Last: packet_size
  if (data[length] != packet_size ) {
    ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[length], packet_size);
    return false;
  }

  // Delete 0x00 at the beginning of the message
  rx_message_.erase(rx_message_.begin());

  // to log the packet
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG,  "Packet received: %s", pretty_cmd.c_str());

  // parse
  parse_status_packet(rx_message_);

  // return false to reset the rx buffer to zero
  return false;
}


// parse the received packets
void NiceBusT4::parse_status_packet (const std::vector<uint8_t> &data) {
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) { // mistake
    ESP_LOGE(TAG,  "The command is not available for this device" );
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // if evt
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG,  "Data string: %s", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "HEX data %s", pretty_data.c_str() );

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) {
      ESP_LOGI(TAG,  "Response received %X ", data[10] );
      switch (data[10]) {
        case TYPE_M:
          switch (data[14]) {
            case SLIDING:   this->class_gate_ = SLIDING;   break;
            case SECTIONAL: this->class_gate_ = SECTIONAL; break;
            case SWING:     this->class_gate_ = SWING;     break;
            case BARRIER:   this->class_gate_ = BARRIER;   break;
            case UPANDOVER: this->class_gate_ = UPANDOVER; break;
          }
          break;

        case INF_IO:
          switch (data[16]) {
            case 0x00:
              ESP_LOGI(TAG, "  The fuse didn't work ");
              break;
            case 0x01:
              ESP_LOGI(TAG, "  Closing limit switch ");
              this->position = COVER_CLOSED;
              break;
            case 0x02:
              ESP_LOGI(TAG, "  Release lever ");
              this->position = COVER_OPEN;
              break;
          }
          this->publish_state_if_changed();
          break;

        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          }
          else {
            this->_max_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Maximum encoder position: %d", this->_max_opn);
          break;

        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Closed gate position: %d", this->_pos_cls);
          break;

        case POS_MAX:
          if (((data[14] << 8) + data[15]) > 0x00) {
            this->_pos_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Open gate position: %d", this->_pos_opn);
          break;

        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
          break;

        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  The gate is open");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  The gate is closed");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  The gate's stopped");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x00:
              ESP_LOGI(TAG, "  The status of the gate is unknown");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x0b:
              ESP_LOGI(TAG, "  The search for provisions has been made");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case STA_OPENING:
              ESP_LOGI(TAG, "  It's opening");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
            case STA_CLOSING:
              ESP_LOGI(TAG, "  Closing in progress");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
          }
          this->publish_state_if_changed();
          break;

        case AUTOCLS:
          this->autocls_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Auto-closing - L1: %s", autocls_flag ? "Yes" : "No");
          break;

        case PH_CLS_ON:
          this->photocls_flag = data[14];
          break;

        case ALW_CLS_ON:
          this->alwayscls_flag = data[14];
          break;
      }
    }

    if ((data[6] == INF) &&  (data[11] == GET - 0x81) && (data[13] == NOERR)) {
      ESP_LOGI(TAG,  "Received incomplete response to request %X, continued at offset %X", data[10], data[12] );
      tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));
    }

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == SET - 0x80) && (data[13] == NOERR)) {
      switch (data[10]) {
        case AUTOCLS:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET));
          break;
        case PH_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET));
          break;
        case ALW_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET));
          break;
      }
    }

    if ((data[6] == INF) && (data[9] == FOR_ALL)  && ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) {
      switch (data[10]) {
        case MAN:
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
            std::vector<uint8_t> wla1 = {0x57,0x4C,0x41,0x31,0x00,0x06,0x57};
            std::vector<uint8_t> ROBUSHSR10 = {0x52,0x4F,0x42,0x55,0x53,0x48,0x53,0x52,0x31,0x30,0x00};
            if (this->product_ == wla1) {
              this->is_walky = true;
            }
            if (this->product_ == ROBUSHSR10) {
              this->is_robus = true;
            }
          }
          break;
        case HWR:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          break;
        case FRM:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          break;
        case DSC:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->description_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          break;
        case WHO:
          if (data[12] == 0x01) {
            if (data[14] == 0x04) { // drive
              this->addr_to[0] = data[4];
              this->addr_to[1] = data[5];
              this->init_ok = true;
            }
            else if (data[14] == 0x0A) { // receiver
              this->addr_oxi[0] = data[4];
              this->addr_oxi[1] = data[5];
              init_device(data[4], data[5], data[14]);
            }
          }
          break;
      }
    }

    if ((data[9] == 0x0A) &&  (data[10] == 0x25) &&  (data[11] == 0x01) &&  (data[12] == 0x0A) &&  (data[13] == NOERR)) {
      ESP_LOGCONFIG(TAG, "Remote number: %X%X%X%X%X, command: %X, button: %X, mode: %X, press counter: %d",
                    vec_data[5], vec_data[4], vec_data[3], vec_data[2], vec_data[8] / 0x10,
                    vec_data[5] / 0x10, vec_data[7] + 0x01, vec_data[6]);
    }

    if ((data[9] == 0x0A) &&  (data[10] == 0x26) &&  (data[11] == 0x41) &&  (data[12] == 0x08) &&  (data[13] == NOERR)) {
      ESP_LOGCONFIG(TAG, "Button %X, remote control number: %X%X%X%X%X%X",
                    vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
    }

  } // if evt
  else if (data[1] > 0x0d) {
    ESP_LOGD(TAG, "RSP package received");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG,  "Data string: %s", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "HEX data %s", pretty_data.c_str() );

    switch (data[9]) {
      case FOR_CU:
        ESP_LOGI(TAG, "Drive controller package");
        switch (data[10] + 0x80) {
          case RUN:
            ESP_LOGI(TAG, "RUN submenu");
            if (data[11] >= 0x80) {
              switch (data[11] - 0x80) {
                case SBS:   ESP_LOGI(TAG, "Command: Step by step"); break;
                case STOP:  ESP_LOGI(TAG, "Command: STOP"); break;
                case OPEN:  ESP_LOGI(TAG, "Command: OPEN");  this->current_operation = COVER_OPERATION_OPENING; break;
                case CLOSE: ESP_LOGI(TAG, "Command: CLOSE"); this->current_operation = COVER_OPERATION_CLOSING; break;
                case P_OPN1: ESP_LOGI(TAG, "Command: Partial opening 1"); break;
                case STOPPED:
                  ESP_LOGI(TAG, "Command: Stopped");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                case ENDTIME:
                  ESP_LOGI(TAG, "Operation terminated due to timeout");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                default:
                  ESP_LOGI(TAG, "Unknown team: %X", data[11]);
              }
            } else {
              switch (data[11]) {
                case STA_OPENING:
                  ESP_LOGI(TAG, "Operation: Opening");
                  this->current_operation = COVER_OPERATION_OPENING;
                  break;
                case STA_CLOSING:
                  ESP_LOGI(TAG, "Operation: Closing");
                  this->current_operation = COVER_OPERATION_CLOSING;
                  break;
                case CLOSED:
                  ESP_LOGI(TAG, "Operation: Closed");
                  this->current_operation = COVER_OPERATION_IDLE;
                  this->position = COVER_CLOSED;
                  break;
                case OPENED:
                  ESP_LOGI(TAG, "Operation: Open");
                  this->current_operation = COVER_OPERATION_IDLE;
                  this->position = COVER_OPEN;
                  if (this->_max_opn == 0) {
                    this->_max_opn = this->_pos_opn = this->_pos_usl;
                    ESP_LOGI(TAG, "Opened position calibrated");
                  }
                  break;
                case STOPPED:
                  ESP_LOGI(TAG, "Operation: Stopped");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                case PART_OPENED:
                  ESP_LOGI(TAG, "Operation: Partially open");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                default:
                  ESP_LOGI(TAG, "Unknown operation: %X", data[11]);
              }
            }
            this->publish_state_if_changed();
            break;

          case STA:
            ESP_LOGI(TAG,  "Status in Motion submenu" );
            switch (data[11]) {
              case STA_OPENING:
              case 0x83:
                ESP_LOGI(TAG, "Motion: Opening" );
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
              case 0x84:
                ESP_LOGI(TAG,  "Motion: Closing" );
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
              case CLOSED:
                ESP_LOGI(TAG,  "Motion: Closed" );
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case OPENED:
                ESP_LOGI(TAG, "Motion: Open");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case STOPPED:
                ESP_LOGI(TAG, "Motion: Stopped");
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              default:
                ESP_LOGI(TAG,  "Motion: %X", data[11] );
            }
            update_position((data[12] << 8) + data[13]);
            break;

          default:
            ESP_LOGI(TAG,  "Submenu %X", data[10] );
        }
        break;

      case CONTROL:
        ESP_LOGI(TAG,  "CONTROL package" );
        break;
      case FOR_ALL:
        ESP_LOGI(TAG,  "Package for all" );
        break;
      case 0x0A:
        ESP_LOGI(TAG,  "Receiver package" );
        break;
      default:
        ESP_LOGI(TAG,  "Menu %X", data[9] );
    }
  }
}

void NiceBusT4::dump_config() {
  ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
  switch (this->class_gate_) {
    case SLIDING:   ESP_LOGCONFIG(TAG, "  Type: Sliding gate"); break;
    case SECTIONAL: ESP_LOGCONFIG(TAG, "  Type: Sectional gate"); break;
    case SWING:     ESP_LOGCONFIG(TAG, "  Type: Swing gate"); break;
    case BARRIER:   ESP_LOGCONFIG(TAG, "  Type: Barrier"); break;
    case UPANDOVER: ESP_LOGCONFIG(TAG, "  Type: Lift and swing gate"); break;
    default:        ESP_LOGCONFIG(TAG, "  Type: Unknown gate, 0x%02X", this->class_gate_);
  }

  ESP_LOGCONFIG(TAG, "  Maximum encoder or timer position: %d", this->_max_opn);
  ESP_LOGCONFIG(TAG, "  Open goal position: %d", this->_pos_opn);
  ESP_LOGCONFIG(TAG, "  Closed gate position: %d", this->_pos_cls);

  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Manufacturer: %s", manuf_str.c_str());

  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Drive: %s", prod_str.c_str());

  std::string hard_str(this->hardware_.begin(), this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Drive hardware: %s", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Drive firmware: %s", firm_str.c_str());

  std::string dsc_str(this->description_.begin(), this->description_.end());
  ESP_LOGCONFIG(TAG, "  Drive description: %s", dsc_str.c_str());

  ESP_LOGCONFIG(TAG, "  Address gateway: 0x%02X%02X", addr_from[0], addr_from[1]);
  ESP_LOGCONFIG(TAG, "  Drive address: 0x%02X%02X", addr_to[0], addr_to[1]);
  ESP_LOGCONFIG(TAG, "  Receiver address: 0x%02X%02X", addr_oxi[0], addr_oxi[1]);

  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Receiver: %s", oxi_prod_str.c_str());

  std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
  ESP_LOGCONFIG(TAG, "  Receiver hardware: %s", oxi_hard_str.c_str());

  std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
  ESP_LOGCONFIG(TAG, "  Receiver firmware: %s", oxi_firm_str.c_str());

  std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
  ESP_LOGCONFIG(TAG, "  Receiver description: %s", oxi_dsc_str.c_str());

  ESP_LOGCONFIG(TAG, "  Auto-closing - L1: %s", autocls_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Close up after the photo - L2: %s", photocls_flag ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Always close - L3: %s", alwayscls_flag ? "Yes" : "No");
}

// management team building
std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], this->addr_from[0], this->addr_from[1]};
  frame.push_back(CMD);
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64);
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);
  return frame;
}

// INF command generation with and without data
std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd,
                                            const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]};
  frame.push_back(INF);
  frame.push_back(0x06 + len);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data);
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end());
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) {
    crc2 = crc2 ^ frame[i];
  }
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);
  return frame;
}

void NiceBusT4::send_raw_cmd(std::string data) {
  std::vector<uint8_t> v_cmd = raw_cmd_prepare(data);
  send_array_cmd(&v_cmd[0], v_cmd.size());
}

// Here we need to add a check for incorrect data from the user
std::vector<uint8_t> NiceBusT4::raw_cmd_prepare(std::string data) {
  data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) { return !(isxdigit(ch)); }), data.end());

  std::vector<uint8_t> frame;
  frame.resize(0);

  for (uint8_t i = 0; i < data.size(); i += 2) {
    std::string sub_str(data, i, 2);
    char hexstoi = (char)std::strtol(&sub_str[0], 0, 16);
    frame.push_back(hexstoi);
  }

  return frame;
}

void NiceBusT4::send_array_cmd(std::vector<uint8_t> data) {
  return send_array_cmd((const uint8_t *) data.data(), data.size());
}
void NiceBusT4::send_array_cmd(const uint8_t *data, size_t len) {
  uint8_t br_ch = 0x00;
  this->parent_->flush();
  this->parent_->set_baud_rate(BAUD_BREAK);
  this->parent_->write_byte(br_ch);
  this->parent_->flush();

  delayMicroseconds(90);

  this->parent_->set_baud_rate(BAUD_WORK);
  this->parent_->write_array(data, len);
  this->parent_->flush();

  std::string pretty_cmd = format_hex_pretty((uint8_t*) &data[0], len);
  ESP_LOGI(TAG, "Posted: %s", pretty_cmd.c_str());
}

void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data,
                             bool data_on, std::string data_command) {
  std::vector<uint8_t> v_to_addr = raw_cmd_prepare(to_addr);
  std::vector<uint8_t> v_whose = raw_cmd_prepare(whose);
  std::vector<uint8_t> v_command = NiceBusT4::raw_cmd_prepare(command);
  std::vector<uint8_t> v_type_command = raw_cmd_prepare(type_command);
  std::vector<uint8_t> v_next_data = raw_cmd_prepare(next_data);
  std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);

  if (data_on) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command,
                                v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
  }
}

void NiceBusT4::set_mcu(std::string command, std::string data_command) {
  std::vector<uint8_t> v_command = raw_cmd_prepare(command);
  std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);
  tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
}

void NiceBusT4::init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device) {
  if (device == FOR_CU) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00));
    if (is_walky)
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00));
  }
  if (device == FOR_OXI) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00));
  }
}

void NiceBusT4::request_position(void) {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;

  int32_t denom = (int32_t) _pos_opn - (int32_t) _pos_cls;
  if (denom == 0) {
    ESP_LOGW(TAG, "Invalid calibration (_pos_opn == _pos_cls). Skipping position update.");
    return;
  }

  position = (_pos_usl - _pos_cls) * 1.0f / denom;
  ESP_LOGI(TAG, "Conditional goal position: %d, position in %%: %.3f", newpos, position);
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();

  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
    ESP_LOGI(TAG, "The desired position has been reached. Stopping the gate");
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
