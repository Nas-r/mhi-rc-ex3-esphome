#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

#include <string>

namespace esphome {
namespace rc_ex3 {

static const uint8_t HEADER_LEN           = 4;
static const uint8_t POS_INDOOR_AIR_TEMP  = 9;
static const uint8_t POS_OUTDOOR_AIR_TEMP = 26;
static const uint8_t POS_RETURN_AIR_TEMP  = 27;
static const uint8_t POS_COMPRESSOR_HZ    = 32;
static const uint8_t POS_INDOOR_FAN_SPEED = 45;

enum class RxState : uint8_t {
  WAITING_FOR_SOF,
  READING_PAYLOAD,
};

class RcEx3Climate : public climate::Climate, public uart::UARTDevice, public PollingComponent {
 public:
  RcEx3Climate() = default;

  void setup() override;
  void loop() override;
  void update() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_op_data_interval(uint32_t minutes) { op_data_interval_minutes_ = minutes; }

  void set_indoor_temperature_sensor(sensor::Sensor *s)    { indoor_temperature_sensor_    = s; }
  void set_outdoor_temperature_sensor(sensor::Sensor *s)   { outdoor_temperature_sensor_   = s; }
  void set_return_air_temperature_sensor(sensor::Sensor *s){ return_air_temperature_sensor_ = s; }
  void set_compressor_frequency_sensor(sensor::Sensor *s)  { compressor_frequency_sensor_  = s; }
  void set_indoor_fan_speed_sensor(sensor::Sensor *s)      { indoor_fan_speed_sensor_      = s; }

 protected:
  void send_command(const char *payload, size_t len);
  void send_status_request();
  void send_operational_data_request(bool second_page = false);
  void send_pending_command();

  void parse_packet(const char *raw, size_t len);
  bool validate_checksum_and_extract_payload_(const char *raw, size_t len, char *payload, size_t payload_size, size_t &payload_len);
  void parse_status_response(const char *buf, size_t len);
  void parse_operational_data(const char *buf, size_t len);

  void apply_wire_fan_mode(char wire_c);
  bool status_matches_desired(char pwr_c, char mode_c, char fan_c, unsigned int raw_temp) const;

  uint8_t calc_checksum(const char *data, size_t len);
  size_t  hex_to_bytes(const char *hex, uint8_t *out, size_t max_out);

  static uint8_t              fan_mode_to_wire(climate::ClimateFanMode mode);
  static uint8_t              custom_fan_mode_to_wire(const std::string &mode);
  static uint8_t              climate_mode_to_wire(climate::ClimateMode mode);
  static climate::ClimateMode wire_to_climate_mode(uint8_t wire_val);

  static const size_t RX_BUF_SIZE = 256;
  char    rx_buf_[RX_BUF_SIZE];
  size_t  rx_len_{0};
  RxState rx_state_{RxState::WAITING_FOR_SOF};

  // Cap bytes drained per loop() call. `while (available())` is unbounded, so a
  // continuously talking or noisy bus would starve the main loop and stall the
  // API. Anything left over is picked up on the next tick.
  static const size_t MAX_RX_BYTES_PER_LOOP = 256;

  // The setpoint range advertised to Home Assistant, and the band a status
  // response must fall inside to be believed.
  static constexpr float MIN_SETPOINT_C = 16.0f;
  static constexpr float MAX_SETPOINT_C = 30.0f;

  // Home Assistant sends one control call per field, and dragging the
  // temperature slider sends one per step. The panel silently drops frames
  // that arrive faster than it can process them, so calls are coalesced into
  // a single frame once they stop arriving, and no two frames are ever sent
  // closer together than MIN_TX_GAP_MS.
  static const uint32_t COMMAND_DEBOUNCE_MS = 400;
  static const uint32_t MIN_TX_GAP_MS       = 150;

  // How long the requested state stays authoritative after a frame goes out,
  // and how many times to re-send before believing the panel instead.
  static const uint32_t COMMAND_SETTLE_MS    = 3000;
  static const uint8_t  MAX_COMMAND_RETRIES  = 2;

  bool     command_pending_{false};
  uint32_t command_dirty_ms_{0};
  uint32_t last_tx_ms_{0};

  // The panel acknowledges a command with RSSL08 and only reports state in an
  // RSSL11, which otherwise arrives on the next scheduled poll — up to a full
  // update_interval away. Waiting for that would expire the settle window on
  // every command, so a status request is issued to confirm actively.
  static const uint32_t CONFIRM_POLL_DELAY_MS = 600;
  static const uint8_t  MAX_CONFIRM_POLLS     = 3;

  bool     awaiting_confirmation_{false};
  uint32_t command_sent_ms_{0};
  uint8_t  command_retries_{0};
  bool     confirm_poll_pending_{false};
  uint8_t  confirm_polls_{0};
  bool     saw_disagreeing_status_{false};

  // What Home Assistant actually asked for, encoded at control() time. A status
  // response arriving during the debounce window rewrites the entity fields, so
  // rebuilding the frame from those at send time would transmit the panel's own
  // value back and silently discard the change.
  uint8_t desired_power_{0};
  uint8_t desired_mode_wire_{0};
  uint8_t desired_fan_wire_{0x07};
  uint8_t desired_temp_wire_{44};

  // The unit answers an op-data request with RSR2 ("not ready") until it is
  // willing to send the blob. Upstream sleeps 500 ms between attempts; this
  // component paces them from loop() instead, and gives up after a bounded
  // number so a sulking unit cannot occupy the bus indefinitely.
  // The unit answers an op-data request with RSR2 ("not ready") until it is
  // willing to send the blob, and the exchange has to run at full speed: it is
  // the number of exchanges that advances it, not elapsed time. Measured on
  // this panel — ~411 exchanges over 8.35 s succeeded, while 39 exchanges
  // paced at 500 ms over 20.3 s left it still not ready. So this is bounded
  // rather than paced, generously enough to complete but not to run away.
  static const uint32_t OP_DATA_HANDSHAKE_MS   = 15000;
  static const uint16_t MAX_OP_DATA_EXCHANGES  = 600;

  uint32_t op_data_started_ms_{0};
  uint16_t op_data_exchanges_{0};

  uint32_t op_data_interval_minutes_{0};
  uint32_t last_op_data_ms_{0};
  bool op_data_pending_{false};
  bool op_data_requested_{false};  // set in update(); cleared when status response chains op_data
  bool rx_overflowed_{false};

  std::string requested_custom_fan_mode_{};

  sensor::Sensor *indoor_temperature_sensor_    {nullptr};
  sensor::Sensor *outdoor_temperature_sensor_   {nullptr};
  sensor::Sensor *return_air_temperature_sensor_{nullptr};
  sensor::Sensor *compressor_frequency_sensor_  {nullptr};
  sensor::Sensor *indoor_fan_speed_sensor_      {nullptr};
};

}  // namespace rc_ex3
}  // namespace esphome
