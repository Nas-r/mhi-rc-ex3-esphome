#include "rc_ex3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rc_ex3 {

static const char *const TAG = "rc_ex3";

// ─── Traits ──────────────────────────────────────────────────────────────────

climate::ClimateTraits RcEx3Climate::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.set_supported_modes({
    climate::CLIMATE_MODE_OFF,
    climate::CLIMATE_MODE_HEAT_COOL,
    climate::CLIMATE_MODE_COOL,
    climate::CLIMATE_MODE_HEAT,
    climate::CLIMATE_MODE_DRY,
    climate::CLIMATE_MODE_FAN_ONLY,
  });
  traits.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO});
  traits.set_visual_min_temperature(16.0f);
  traits.set_visual_max_temperature(30.0f);
  traits.set_visual_temperature_step(0.5f);
  return traits;
}

// ─── Lifecycle ───────────────────────────────────────────────────────────────

void RcEx3Climate::setup() {
  this->set_supported_custom_fan_modes({"1", "2", "3", "4"});
  this->mode                = climate::CLIMATE_MODE_OFF;
  this->target_temperature  = 22.0f;
  this->current_temperature = NAN;
}

void RcEx3Climate::update() {
  send_status_request();

  op_data_requested_ = false;
  if (op_data_interval_minutes_ == 0)
    return;

  const uint32_t now = millis();
  const uint32_t interval_ms = op_data_interval_minutes_ * 60000UL;
  if (last_op_data_ms_ == 0 || (now - last_op_data_ms_) >= interval_ms)
    op_data_requested_ = true;
}

// ─── Serial RX loop ──────────────────────────────────────────────────────────

void RcEx3Climate::loop() {
  while (this->available()) {
    uint8_t c;
    if (!this->read_byte(&c))
      break;

    if (rx_state_ == RxState::WAITING_FOR_SOF) {
      if (c == 0x02) {
        rx_len_ = 0;
        rx_overflowed_ = false;
        rx_state_ = RxState::READING_PAYLOAD;
      }
    } else {
      if (c == 0x03) {
        if (rx_overflowed_) {
          ESP_LOGW(TAG, "rx frame dropped after overflow");
        } else {
          rx_buf_[rx_len_] = '\0';
          parse_packet(rx_buf_, rx_len_);
        }
        rx_state_ = RxState::WAITING_FOR_SOF;
        rx_len_   = 0;
      } else if (rx_len_ < RX_BUF_SIZE - 1) {
        rx_buf_[rx_len_++] = static_cast<char>(c);
      } else {
        rx_overflowed_ = true;
      }
    }
  }

  const uint32_t now = millis();

  // Send the op_data request once the status response has been received,
  // separated from the previous frame to avoid overlapping Tx.
  if (op_data_pending_ && (now - last_tx_ms_) >= MIN_TX_GAP_MS) {
    op_data_pending_ = false;
    send_operational_data_request(false);
    return;
  }

  // One coalesced control frame, sent only once Home Assistant has stopped
  // calling and the bus has been quiet long enough for the panel to keep up.
  if (command_pending_ && (now - command_dirty_ms_) >= COMMAND_DEBOUNCE_MS &&
      (now - last_tx_ms_) >= MIN_TX_GAP_MS) {
    command_pending_ = false;
    send_pending_command();
  }
}

// ─── HA control call ─────────────────────────────────────────────────────────

void RcEx3Climate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();

  // A call carries either a built-in fan mode or a custom one. The setters
  // below each clear the other, so the two can never disagree and selecting
  // Auto can always get back out of a custom speed.
  auto custom_fan_mode = call.get_custom_fan_mode();
  if (!custom_fan_mode.empty()) {
    this->requested_custom_fan_mode_.assign(custom_fan_mode.c_str(), custom_fan_mode.size());
    this->set_custom_fan_mode_(custom_fan_mode);
  } else if (call.get_fan_mode().has_value()) {
    this->requested_custom_fan_mode_.clear();
    this->set_fan_mode_(*call.get_fan_mode());
  }

  // Encode the request now rather than at send time. A status response arriving
  // during the debounce window rewrites the entity fields, and rebuilding the
  // frame from those would transmit the panel's own value straight back.
  this->desired_power_     = (this->mode == climate::CLIMATE_MODE_OFF) ? 0 : 1;
  this->desired_mode_wire_ = climate_mode_to_wire(this->mode);
  this->desired_fan_wire_  = custom_fan_mode_to_wire(this->requested_custom_fan_mode_);
  this->desired_temp_wire_ = static_cast<uint8_t>(this->target_temperature * 2.0f);

  // Don't transmit yet — mark the state dirty and let loop() send one frame
  // carrying every field once the calls stop arriving. Publish immediately so
  // the Home Assistant UI stays responsive while that settles.
  this->command_pending_   = true;
  this->command_dirty_ms_  = millis();
  this->publish_state();
}

void RcEx3Climate::send_pending_command() {
  char buf[64];
  size_t len = snprintf(buf, sizeof(buf),
    "RSSL13FF0001%.2x02%.2x03%.2x04FF0503%.2x06FF0FFF43FF",
    this->desired_power_, this->desired_mode_wire_,
    this->desired_fan_wire_, this->desired_temp_wire_);

  ESP_LOGI(TAG, "tx → power=%d mode=%d fan=0x%02x temp_wire=%d (%.1f°C)",
           this->desired_power_, this->desired_mode_wire_, this->desired_fan_wire_,
           this->desired_temp_wire_, this->desired_temp_wire_ * 0.5f);

  send_command(buf, len);
}

// ─── Packet dispatch ─────────────────────────────────────────────────────────

void RcEx3Climate::parse_packet(const char *raw, size_t len) {
  char payload[256];
  size_t payload_len = 0;
  if (!validate_checksum_and_extract_payload_(raw, len, payload, sizeof(payload), payload_len))
    return;

  char buf[256];
  size_t buflen = 0;
  bool started = false;

  for (size_t i = 0; i < payload_len && buflen < sizeof(buf) - 1; i++) {
    uint8_t c = static_cast<uint8_t>(payload[i]);
    if (!started) {
      if (payload[i] == 'R') {
        buf[buflen++] = payload[i];
        started = true;
      }
    } else if (c >= 32 && c < 127) {
      buf[buflen++] = payload[i];
    }
  }
  buf[buflen] = '\0';

  if (buflen < 5)
    return;

  ESP_LOGV(TAG, "rx: %s", buf);

  // RSSL1x → climate status; queue op_data only if this update() cycle requested it
  if (buf[0] == 'R' && buf[1] == 'S' && buf[2] == 'S' && buf[3] == 'L' && buf[4] == '1') {
    parse_status_response(buf, buflen);
    if (op_data_requested_) {
      op_data_requested_ = false;
      op_data_pending_   = true;
    }
    return;
  }

  // RSR → operational data handshake / response
  if (buf[0] == 'R' && buf[1] == 'S' && buf[2] == 'R') {
    if (buf[3] == '2') {
      // Unit not yet ready; echo RSR2 immediately and it will eventually respond RSR1
      send_operational_data_request(true);
    } else if (buf[3] == '1') {
      parse_operational_data(buf, buflen);
    }
    return;
  }

  ESP_LOGD(TAG, "rx unhandled: %s", buf);
}


bool RcEx3Climate::validate_checksum_and_extract_payload_(const char *raw, size_t len, char *payload,
                                                          size_t payload_size, size_t &payload_len) {
  payload_len = 0;
  if (len < 3) {
    ESP_LOGW(TAG, "rx frame too short for checksum");
    return false;
  }

  const size_t body_len = len - 2;
  const char rx_hi = raw[body_len];
  const char rx_lo = raw[body_len + 1];
  if (!isxdigit(static_cast<uint8_t>(rx_hi)) || !isxdigit(static_cast<uint8_t>(rx_lo))) {
    ESP_LOGW(TAG, "rx frame missing checksum hex");
    return false;
  }

  char rx_sum_hex[3] = {rx_hi, rx_lo, '\0'};
  uint8_t rx_sum = static_cast<uint8_t>(strtol(rx_sum_hex, nullptr, 16));
  uint8_t calc_sum = calc_checksum(raw, body_len);
  if (rx_sum != calc_sum) {
    ESP_LOGW(TAG, "rx checksum mismatch: got=%02X expected=%02X", rx_sum, calc_sum);
    return false;
  }

  payload_len = (body_len < (payload_size - 1)) ? body_len : (payload_size - 1);
  memcpy(payload, raw, payload_len);
  payload[payload_len] = '\0';
  return true;
}

// ─── Status response parser ───────────────────────────────────────────────────
//
//   [0-3]  "RSSL"
//   [4]    '1'
//   [13]   power  ('0'=off, '1'=on)
//   [17]   mode   ('0'=auto,'1'=dry,'2'=cool,'3'=fan,'4'=heat)
//   [21]   fan    ('0'=spd1,'1'=spd2,'2'=spd3,'6'=spd4,other=auto)
//   [30-31] temp  (2 hex chars, value * 0.5 = °C)

void RcEx3Climate::parse_status_response(const char *buf, size_t len) {
  if (len < 32)
    return;

  char pwr_c  = buf[13];
  char mode_c = buf[17];
  char fan_c  = buf[21];

  char tmp[3] = {buf[30], buf[31], '\0'};
  unsigned int raw_temp = static_cast<unsigned int>(strtol(tmp, nullptr, 16));
  float temp_c = raw_temp * 0.5f;

  bool is_on = (pwr_c == '1');
  climate::ClimateMode new_mode = is_on ? wire_to_climate_mode(mode_c - '0') : climate::CLIMATE_MODE_OFF;

  ESP_LOGD(TAG, "status: power=%c mode=%c fan=%c temp=%.1f°C", pwr_c, mode_c, fan_c, temp_c);

  this->mode               = new_mode;
  apply_wire_fan_mode(fan_c);
  this->target_temperature = temp_c;
  if (std::isnan(this->current_temperature) && indoor_temperature_sensor_ &&
      !std::isnan(indoor_temperature_sensor_->state)) {
    this->current_temperature = indoor_temperature_sensor_->state;
  }
  this->publish_state();
}

// Map a wire fan-speed character onto the entity. Speeds 1–4 are exposed as
// custom fan modes, so they have to be published through set_custom_fan_mode_()
// — fan_mode only ever carries the built-in Auto. Publishing a fan_mode that
// traits() doesn't advertise leaves Home Assistant showing a stale selection.
void RcEx3Climate::apply_wire_fan_mode(char wire_c) {
  const char *custom = nullptr;
  switch (wire_c) {
    case '0': custom = "1"; break;
    case '1': custom = "2"; break;
    case '2': custom = "3"; break;
    case '6': custom = "4"; break;
    default: break;  // '7' and anything unrecognised → Auto
  }

  if (custom != nullptr) {
    this->requested_custom_fan_mode_ = custom;
    this->set_custom_fan_mode_(custom);
  } else {
    this->requested_custom_fan_mode_.clear();
    this->set_fan_mode_(climate::CLIMATE_FAN_AUTO);
  }
}

// ─── Operational data parser ──────────────────────────────────────────────────
//
// Request:  RSR10000E8  → page 1
// Response: RSR1<hex-encoded binary blob>
// After HEADER_LEN=4 ("RSR1"), the rest is hex pairs encoding raw bytes.

void RcEx3Climate::parse_operational_data(const char *buf, size_t len) {
  const char *hex_data = buf + HEADER_LEN;
  uint8_t data[256] = {};
  size_t data_len = hex_to_bytes(hex_data, data, sizeof(data));

  if (data_len < (POS_INDOOR_FAN_SPEED - HEADER_LEN + 1)) {
    ESP_LOGW(TAG, "op-data too short (%d bytes)", (int)data_len);
    return;
  }

  auto idx = [](uint8_t pos) { return pos - HEADER_LEN; };

  float indoor_air  = static_cast<float>(static_cast<int8_t>(data[idx(POS_INDOOR_AIR_TEMP)]));
  float outdoor_air = static_cast<float>(static_cast<uint8_t>(data[idx(POS_OUTDOOR_AIR_TEMP)]) / 4 - 22);
  float return_air  = static_cast<float>(data[idx(POS_RETURN_AIR_TEMP)]) / 10.0f;
  uint8_t comp_hz   = data[idx(POS_COMPRESSOR_HZ)];
  uint8_t in_fan    = data[idx(POS_INDOOR_FAN_SPEED)];

  ESP_LOGD(TAG, "op-data raw: indoor=%d outdoor=%d return=%d",
           data[idx(POS_INDOOR_AIR_TEMP)], data[idx(POS_OUTDOOR_AIR_TEMP)], data[idx(POS_RETURN_AIR_TEMP)]);
  ESP_LOGI(TAG, "op-data → indoor=%.1f°C outdoor=%.1f°C return=%.1f°C comp=%dHz fan=%d",
           indoor_air, outdoor_air, return_air, comp_hz, in_fan);

  if (indoor_temperature_sensor_)     indoor_temperature_sensor_->publish_state(indoor_air);
  if (outdoor_temperature_sensor_)    outdoor_temperature_sensor_->publish_state(outdoor_air);
  if (return_air_temperature_sensor_) return_air_temperature_sensor_->publish_state(return_air);
  if (compressor_frequency_sensor_)   compressor_frequency_sensor_->publish_state(comp_hz);
  if (indoor_fan_speed_sensor_)       indoor_fan_speed_sensor_->publish_state(in_fan);

  last_op_data_ms_ = millis();
  this->current_temperature = indoor_air;
  this->publish_state();
}

// ─── Packet send helpers ─────────────────────────────────────────────────────

uint8_t RcEx3Climate::calc_checksum(const char *data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++)
    sum += static_cast<uint8_t>(data[i]);
  return sum;
}

void RcEx3Climate::send_command(const char *payload, size_t len) {
  uint8_t sum = calc_checksum(payload, len);
  char hex_sum[3];
  snprintf(hex_sum, sizeof(hex_sum), "%02X", sum);

  this->write_byte(0x02);
  for (size_t i = 0; i < len; i++)
    this->write_byte(static_cast<uint8_t>(payload[i]));
  this->write_byte(static_cast<uint8_t>(hex_sum[0]));
  this->write_byte(static_cast<uint8_t>(hex_sum[1]));
  this->write_byte(0x03);
  last_tx_ms_ = millis();
}

void RcEx3Climate::send_status_request() {
  const char *query = "RSSL12FF0001FF02FF03FF04FF05FF06FF0FFF43FF25";
  this->write_byte(0x02);
  for (const char *p = query; *p; p++)
    this->write_byte(static_cast<uint8_t>(*p));
  this->write_byte(0x03);
  last_tx_ms_ = millis();
}

void RcEx3Climate::send_operational_data_request(bool second_page) {
  const char *query = second_page ? "RSR20000E9" : "RSR10000E8";
  this->write_byte(0x02);
  for (const char *p = query; *p; p++)
    this->write_byte(static_cast<uint8_t>(*p));
  this->write_byte(0x03);
  last_tx_ms_ = millis();
}

// ─── Encoding helpers ─────────────────────────────────────────────────────────

uint8_t RcEx3Climate::climate_mode_to_wire(climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT_COOL: return 0;
    case climate::CLIMATE_MODE_DRY:       return 1;
    case climate::CLIMATE_MODE_COOL:      return 2;
    case climate::CLIMATE_MODE_FAN_ONLY:  return 3;
    case climate::CLIMATE_MODE_HEAT:      return 4;
    default:                              return 0;
  }
}

climate::ClimateMode RcEx3Climate::wire_to_climate_mode(uint8_t v) {
  switch (v) {
    case 0: return climate::CLIMATE_MODE_HEAT_COOL;
    case 1: return climate::CLIMATE_MODE_DRY;
    case 2: return climate::CLIMATE_MODE_COOL;
    case 3: return climate::CLIMATE_MODE_FAN_ONLY;
    case 4: return climate::CLIMATE_MODE_HEAT;
    default: return climate::CLIMATE_MODE_HEAT_COOL;
  }
}

uint8_t RcEx3Climate::fan_mode_to_wire(climate::ClimateFanMode) {
  return 0x07;
}

uint8_t RcEx3Climate::custom_fan_mode_to_wire(const std::string &mode) {
  if (mode == "1") return 0x00;
  if (mode == "2") return 0x01;
  if (mode == "3") return 0x02;
  if (mode == "4") return 0x06;
  return 0x07;  // empty or unrecognised → Auto
}

size_t RcEx3Climate::hex_to_bytes(const char *hex, uint8_t *out, size_t max_out) {
  size_t count = 0;
  char tmp[3] = {0};
  while (hex[0] && hex[1] && count < max_out) {
    if (!isxdigit(static_cast<uint8_t>(hex[0])) || !isxdigit(static_cast<uint8_t>(hex[1])))
      break;
    tmp[0] = hex[0];
    tmp[1] = hex[1];
    out[count++] = static_cast<uint8_t>(strtol(tmp, nullptr, 16));
    hex += 2;
  }
  return count;
}

}  // namespace rc_ex3
}  // namespace esphome
