/*
  Nice BusT4
  Data exchange via UART at 19200 8N1
  A break of 519µs (10 bits) is sent before the data packet.
  The contents of the packet that have been deciphered are described in the structure packet_cmd_body_t.

  For Oview, 80 is always added to the address.
  The gate controller address remains unchanged.

Connection:

BusT4                       ESP32

Device Panel           Rx Tx GND
9  7  5  3  1
10 8  6  4  2
Cable slot
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V

From the manual nice_dmbm_integration_protocol.pdf:

• ADR: This is the NICE network address where the devices you want to control are located. The value can range from 1 to 63 (1 to 3F).
  This value must be in HEX. If the recipient is an integration module on a DIN bar, this value is 0 (adr = 0). If the recipient
  is an intelligent motor, the value is 1 (adr = 1).
• EPT: This is the address of the Nice motor within the network ADR. The value can range from 1 to 127 and must be in HEX.
• CMD: This is the command you want to send to the destination (ADR, EPT).
• PRF: Profile setting command.
• FNC: This is the function you want to send to the destination (ADR, EPT).
• EVT: This is an event sent to the destination (ADR, EPT).
*/

#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <queue>
#include <string>
#include <vector>

#include "consts.h"

static const uint32_t BAUD_BREAK = 9200; /* Baud rate for the long pulse before the packet */
static const uint32_t BAUD_WORK = 19200; /* Working baud rate */
static const uint8_t START_CODE = 0x55;  /* Start byte of the packet */

static const float CLOSED_POSITION_THRESHOLD = 0.007f;
static const uint32_t POSITION_UPDATE_INTERVAL = 500;

namespace esphome {
namespace bus_t4 {

/* Command to be executed.
   The 11th byte of the packet CMD
   Used in requests and responses */
enum control_cmd : uint8_t {
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - pedestrian mode */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
  RSP = 0x19,    /* Interface response confirming command reception */
  EVT = 0x29,    /* Interface response sending requested information */

  P_OPN4 = 0x0b,    /* Partial opening 4 - Collective */
  P_OPN5 = 0x0c,    /* Partial opening 5 - Priority step-by-step */
  P_OPN6 = 0x0d,    /* Partial opening 6 - Open and lock */
  UNLK_OPN = 0x19,  /* Unlock and open */
  CLS_LOCK = 0x0E,  /* Close and lock */
  UNLCK_CLS = 0x1A, /* Unlock and close */
  LOCK = 0x0F,      /* Lock */
  UNLOCK = 0x10,    /* Unlock */
  LIGHT_TIMER = 0x11, /* Lighting timer */
  LIGHT_SW = 0x12,    /* Light on/off */
  HOST_SBS = 0x13,     /* Master SBS */
  HOST_OPN = 0x14,     /* Master open */
  HOST_CLS = 0x15,     /* Master close */
  SLAVE_SBS = 0x16,    /* Slave SBS */
  SLAVE_OPN = 0x17,    /* Slave open */
  SLAVE_CLS = 0x18,    /* Slave close */
  AUTO_ON = 0x1B,      /* Auto-opening active */
  AUTO_OFF = 0x1C,     /* Auto-opening inactive */
};

class NiceBusT4 : public cover::Cover, public Component, public uart::UARTDevice {
 public:
  NiceBusT4() = default;

  // Drive settings
  bool autocls_flag;     // Auto-closing - L1
  bool photocls_flag;    // Close after photo - L2
  bool alwayscls_flag;   // Always close - L3
  bool init_ok = false;  // Drive detection at startup
  bool is_walky = false; // For Walky, the position request command differs
  bool is_robus = false; // For Robus, no need to periodically request position

  void setup() override;
  void loop() override;
  void dump_config() override; // Log equipment information

  void send_raw_cmd(std::string data);
  void send_cmd(uint8_t data) { this->tx_buffer_.push(gen_control_cmd(data)); }
  void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data,
                    bool data_on, std::string data_command);
  void set_mcu(std::string command, std::string data_command);

  void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }

  cover::CoverTraits get_traits() override;

  // ✅ importante: public en ESPHome moderno
  void control(const cover::CoverCall &call) override;

 protected:
  void send_command_(const uint8_t *data, uint8_t len);
  void request_position(void);       // Запрос условного текущего положения привода
  void update_position(uint16_t newpos);  // Обновление текущего положения привода

  uint32_t last_position_time{0}; // Время последнего обновления текущего положения
  uint32_t update_interval_{500};
  uint32_t last_update_{0};
  uint32_t last_uart_byte_{0};

  cover::CoverOperation last_published_op; // Последние опубликованные состояние и положение
  float last_published_pos{-1};

  void publish_state_if_changed(void);

  uint8_t position_hook_type{IGNORE}; // Флаг и позиция установки заданного положения привода
  uint16_t position_hook_value;

  uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over

  bool init_cu_flag = false;
  bool init_oxi_flag = false;

  // Variables for UART
  uint16_t _max_opn = 0;      // Maximum position of the encoder or timer
  uint16_t _pos_opn = 2048;   // Opening position of the encoder or timer, not for all drives
  uint16_t _pos_cls = 0;      // Closing position of the encoder or timer, not for all drives
  uint16_t _pos_usl = 0;      // Conditional current position of the encoder or timer, not for all drives

  // Settings for the generated packet header
  uint8_t addr_from[2] = {0x00, 0x66}; // Sender of the packet, address of the bust4 gateway
  uint8_t addr_to[2] = {0x00, 0xff};   // ✅ init to broadcast / sane default
  uint8_t addr_oxi[2] = {0x00, 0x0a};  // ✅ init to common receiver default

  std::vector<uint8_t> raw_cmd_prepare(std::string data); // подготовка введенных пользователем данных для возможности отправки

  // генерация inf команд
  std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd,
                                   const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len); // все поля
  std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {
    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0);
  } // для команд без данных
  std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data,
                                   std::vector<uint8_t> data) {
    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());
  } // для команд c данными
  std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd,
                                   const uint8_t run_cmd, const uint8_t next_data) {
    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);
  } // для команд с адресом и без данных

  // генерация cmd команд
  std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);

  void init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device);
  void send_array_cmd(std::vector<uint8_t> data);
  void send_array_cmd(const uint8_t *data, size_t len);

  void parse_status_packet(const std::vector<uint8_t> &data); // разбираем пакет статуса

  void handle_char_(uint8_t c);      // обработчик полученного байта
  void handle_datapoint_(const uint8_t *buffer, size_t len); // (declarado, no usado aquí)
  bool validate_message_();          // функция проверки полученного сообщения

  std::vector<uint8_t> rx_message_;                  // здесь побайтно накапливается принятое сообщение
  std::queue<std::vector<uint8_t>> tx_buffer_;       // очередь команд для отправки
  bool ready_to_tx_{true};                           // флаг возможности отправлять команды

  std::vector<uint8_t> manufacturer_ = {0x55, 0x55}; // при инициализации неизвестный производитель
  std::vector<uint8_t> product_;
  std::vector<uint8_t> hardware_;
  std::vector<uint8_t> firmware_;
  std::vector<uint8_t> description_;
  std::vector<uint8_t> oxi_product;
  std::vector<uint8_t> oxi_hardware;
  std::vector<uint8_t> oxi_firmware;
  std::vector<uint8_t> oxi_description;
};

}  // namespace bus_t4
}  // namespace esphome
