#include "daikin_ducted.h"
#include "homebus_rmt.h"
#include "p1p2_pkt.h"

#include <cstring>

namespace esphome
{
  namespace daikin_ducted
  {

    static const char *const TAG = "daikin_ducted.climate";

    // ==========================================================================
    // CRC Calculation
    // ==========================================================================

    static const uint8_t CRC_TABLE[256] = {
        0x00, 0xd0, 0x13, 0xc3, 0x26, 0xf6, 0x35, 0xe5, 0x4c, 0x9c, 0x5f, 0x8f,
        0x6a, 0xba, 0x79, 0xa9, 0x98, 0x48, 0x8b, 0x5b, 0xbe, 0x6e, 0xad, 0x7d,
        0xd4, 0x04, 0xc7, 0x17, 0xf2, 0x22, 0xe1, 0x31, 0x83, 0x53, 0x90, 0x40,
        0xa5, 0x75, 0xb6, 0x66, 0xcf, 0x1f, 0xdc, 0x0c, 0xe9, 0x39, 0xfa, 0x2a,
        0x1b, 0xcb, 0x08, 0xd8, 0x3d, 0xed, 0x2e, 0xfe, 0x57, 0x87, 0x44, 0x94,
        0x71, 0xa1, 0x62, 0xb2, 0xb5, 0x65, 0xa6, 0x76, 0x93, 0x43, 0x80, 0x50,
        0xf9, 0x29, 0xea, 0x3a, 0xdf, 0x0f, 0xcc, 0x1c, 0x2d, 0xfd, 0x3e, 0xee,
        0x0b, 0xdb, 0x18, 0xc8, 0x61, 0xb1, 0x72, 0xa2, 0x47, 0x97, 0x54, 0x84,
        0x36, 0xe6, 0x25, 0xf5, 0x10, 0xc0, 0x03, 0xd3, 0x7a, 0xaa, 0x69, 0xb9,
        0x5c, 0x8c, 0x4f, 0x9f, 0xae, 0x7e, 0xbd, 0x6d, 0x88, 0x58, 0x9b, 0x4b,
        0xe2, 0x32, 0xf1, 0x21, 0xc4, 0x14, 0xd7, 0x07, 0xd9, 0x09, 0xca, 0x1a,
        0xff, 0x2f, 0xec, 0x3c, 0x95, 0x45, 0x86, 0x56, 0xb3, 0x63, 0xa0, 0x70,
        0x41, 0x91, 0x52, 0x82, 0x67, 0xb7, 0x74, 0xa4, 0x0d, 0xdd, 0x1e, 0xce,
        0x2b, 0xfb, 0x38, 0xe8, 0x5a, 0x8a, 0x49, 0x99, 0x7c, 0xac, 0x6f, 0xbf,
        0x16, 0xc6, 0x05, 0xd5, 0x30, 0xe0, 0x23, 0xf3, 0xc2, 0x12, 0xd1, 0x01,
        0xe4, 0x34, 0xf7, 0x27, 0x8e, 0x5e, 0x9d, 0x4d, 0xa8, 0x78, 0xbb, 0x6b,
        0x6c, 0xbc, 0x7f, 0xaf, 0x4a, 0x9a, 0x59, 0x89, 0x20, 0xf0, 0x33, 0xe3,
        0x06, 0xd6, 0x15, 0xc5, 0xf4, 0x24, 0xe7, 0x37, 0xd2, 0x02, 0xc1, 0x11,
        0xb8, 0x68, 0xab, 0x7b, 0x9e, 0x4e, 0x8d, 0x5d, 0xef, 0x3f, 0xfc, 0x2c,
        0xc9, 0x19, 0xda, 0x0a, 0xa3, 0x73, 0xb0, 0x60, 0x85, 0x55, 0x96, 0x46,
        0x77, 0xa7, 0x64, 0xb4, 0x51, 0x81, 0x42, 0x92, 0x3b, 0xeb, 0x28, 0xf8,
        0x1d, 0xcd, 0x0e, 0xde};

    static uint8_t calculate_crc(const uint8_t *data, size_t length)
    {
      uint8_t crc = 0;
      for (size_t i = 0; i < length; i++)
      {
        crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
      }
      return crc;
    }

    static void finalize_and_send(HomebusRMT &bus, uint8_t *packet, size_t length)
    {
      packet[length - 1] = calculate_crc(packet, length - 1);
      bus.write_bytes(packet, length);
    }

    // ==========================================================================
    // Protocol Conversion Helpers
    // ==========================================================================

    static uint8_t climate_mode_to_p1p2(climate::ClimateMode mode)
    {
      switch (mode)
      {
      case climate::CLIMATE_MODE_FAN_ONLY:
        return OperatingMode::FAN_ONLY;
      case climate::CLIMATE_MODE_HEAT:
        return OperatingMode::HEAT;
      case climate::CLIMATE_MODE_COOL:
        return OperatingMode::COOL;
      case climate::CLIMATE_MODE_HEAT_COOL:
        return OperatingMode::AUTO;
      case climate::CLIMATE_MODE_DRY:
        return OperatingMode::DRY;
      default:
        return OperatingMode::AUTO;
      }
    }

    static climate::ClimateMode p1p2_to_climate_mode(uint8_t mode)
    {
      switch (mode)
      {
      case OperatingMode::FAN_ONLY:
        return climate::CLIMATE_MODE_FAN_ONLY;
      case OperatingMode::HEAT:
        return climate::CLIMATE_MODE_HEAT;
      case OperatingMode::COOL:
        return climate::CLIMATE_MODE_COOL;
      case OperatingMode::AUTO:
        return climate::CLIMATE_MODE_HEAT_COOL;
      case OperatingMode::DRY:
        return climate::CLIMATE_MODE_DRY;
      default:
        return climate::CLIMATE_MODE_OFF;
      }
    }

    static climate::ClimateAction p1p2_to_climate_action(uint8_t action)
    {
      switch (action)
      {
      case ActionState::HEATING:
        return climate::CLIMATE_ACTION_HEATING;
      case ActionState::COOLING:
        return climate::CLIMATE_ACTION_COOLING;
      case ActionState::IDLE:
      default:
        return climate::CLIMATE_ACTION_FAN;
      }
    }

    static uint8_t climate_fan_to_p1p2(climate::ClimateFanMode fan_mode)
    {
      switch (fan_mode)
      {
      case climate::CLIMATE_FAN_LOW:
        return FanSpeed::LOW;
      case climate::CLIMATE_FAN_MEDIUM:
        return FanSpeed::MEDIUM;
      case climate::CLIMATE_FAN_HIGH:
        return FanSpeed::HIGH;
      default:
        return FanSpeed::MEDIUM;
      }
    }

    static climate::ClimateFanMode p1p2_to_climate_fan(uint8_t speed)
    {
      switch (speed & FanSpeed::MASK)
      {
      case FanSpeed::LOW:
        return climate::CLIMATE_FAN_LOW;
      case FanSpeed::MEDIUM:
        return climate::CLIMATE_FAN_MEDIUM;
      case FanSpeed::HIGH:
        return climate::CLIMATE_FAN_HIGH;
      default:
        return climate::CLIMATE_FAN_MEDIUM;
      }
    }

    static int p1p2_fan_to_speed_level(uint8_t speed)
    {
      switch (speed & FanSpeed::MASK)
      {
      case FanSpeed::LOW:
        return 1;
      case FanSpeed::MEDIUM:
        return 2;
      case FanSpeed::HIGH:
        return 3;
      default:
        return 2;
      }
    }

    // ==========================================================================
    // Packet Handlers
    // ==========================================================================

    void DaikinClimate::handle_temperature_packet(const uint8_t *buffer, bool is_response)
    {
      size_t temp_offset = TemperaturePacket::INDOOR_TEMP_OFFSET;
      float temperature = fixed_point_to_float(buffer[temp_offset], buffer[temp_offset + 1]);

      if (is_response)
      {
        // Response packet: outdoor intake temperature
        ESP_LOGI(TAG, "Outdoor intake: %.2fC", temperature);
        if (this->outdoor_intake_temperature_sensor_ != nullptr)
        {
          this->outdoor_intake_temperature_sensor_->publish_state(temperature);
        }

        // Coolant temperature at different offset
        float coolant_temp = fixed_point_to_float(
            buffer[TemperaturePacket::COOLANT_TEMP_OFFSET],
            buffer[TemperaturePacket::COOLANT_TEMP_OFFSET + 1]);
        if (this->coolant_temperature_sensor_ != nullptr)
        {
          this->coolant_temperature_sensor_->publish_state(coolant_temp);
        }
      }
      else
      {
        // Request packet: indoor temperature
        ESP_LOGI(TAG, "Indoor: %.2fC", temperature);
        this->current_temperature = temperature;

        if (fabs(this->last_temp_state - this->current_temperature) > TEMP_PUBLISH_THRESHOLD)
        {
          this->publish_state();
          this->sync_auxiliary_climate();
          this->last_temp_state = this->current_temperature;
        }

        if (this->indoor_temperature_sensor_ != nullptr)
        {
          this->indoor_temperature_sensor_->publish_state(temperature);
        }
      }
    }

    void DaikinClimate::handle_control_packet(const uint8_t *payload)
    {
      ESP_LOGI(TAG, "Control message (0x38)");

      bool state_changed = false;

      // Build response packet, starting with current values from request
      ControlResponse response = {
          .header = {Direction::RESPONSE, Address::AUX_CONTROLLER, PacketType::OPERATION_CONTROL},
          .power_status = static_cast<uint8_t>(payload[ControlRequest::POWER_STATUS] & PowerState::ON),
          .operating_mode = payload[ControlRequest::OPERATING_MODE],
          .cooling_setpoint = payload[ControlRequest::COOLING_SETPOINT],
          .reserved_6 = 0x00,
          .cooling_fan_speed = payload[ControlRequest::COOLING_FAN_SPEED],
          .reserved_8 = 0x00,
          .heating_setpoint = payload[ControlRequest::HEATING_SETPOINT],
          .reserved_10 = 0x00,
          .heating_fan_speed = payload[ControlRequest::HEATING_FAN_SPEED],
          .unknown_12 = payload[ControlRequest::UNKNOWN_11],
          .reserved_13 = 0x00,
          .reserved_14 = 0x00,
          .reserved_15 = 0x00,
          .status_flags = payload[ControlRequest::STATUS_FLAGS],
          .reserved_17 = 0x00,
          .reserved_18 = 0x00,
          .reserved_19 = 0x00,
          .crc = 0xFF};

      // Handle mode changes
      if (this->mode_updated)
      {
        response.power_status = (this->mode != climate::CLIMATE_MODE_OFF) ? PowerState::ON : PowerState::OFF;
        response.operating_mode = climate_mode_to_p1p2(this->mode);
        response.reserved_6 = FanSpeed::CHANGED_FLAG; // Signal mode change
        this->mode_updated = false;
        state_changed = true;
      }
      else
      {
        // Read mode from device
        climate::ClimateMode new_mode;
        if (payload[ControlRequest::POWER_STATUS] == PowerState::OFF)
        {
          new_mode = climate::CLIMATE_MODE_OFF;
        }
        else
        {
          new_mode = p1p2_to_climate_mode(payload[ControlRequest::OPERATING_MODE]);
        }

        if (new_mode != this->mode)
        {
          this->mode = new_mode;
          state_changed = true;
        }

        // Sync fan entity state
        if (this->fan_ != nullptr)
        {
          bool fan_on = (new_mode == climate::CLIMATE_MODE_FAN_ONLY);
          if (this->fan_->state != fan_on)
          {
            this->fan_->state = fan_on;
            this->fan_->publish_state();
          }
        }
      }

      // Handle action state (always read from device)
      climate::ClimateAction new_action;
      if (payload[ControlRequest::POWER_STATUS] == PowerState::OFF)
      {
        new_action = climate::CLIMATE_ACTION_OFF;
      }
      else
      {
        new_action = p1p2_to_climate_action(payload[ControlRequest::CURRENT_ACTION]);
      }

      if (new_action != this->action)
      {
        this->action = new_action;
        state_changed = true;
      }

      // Handle fan speed changes
      if (this->fan_updated)
      {
        uint8_t fan_speed = climate_fan_to_p1p2(this->fan_mode.value_or(climate::CLIMATE_FAN_MEDIUM));
        uint8_t speed_with_flag = fan_speed | FanSpeed::CHANGED_FLAG;

        response.cooling_fan_speed = (payload[ControlRequest::COOLING_FAN_SPEED] & ~FanSpeed::MASK) | speed_with_flag;
        response.heating_fan_speed = (payload[ControlRequest::HEATING_FAN_SPEED] & ~FanSpeed::MASK) | speed_with_flag;

        this->fan_updated = false;
        state_changed = true;
      }
      else
      {
        // Read fan speed from device
        auto new_fan_mode = p1p2_to_climate_fan(payload[ControlRequest::COOLING_FAN_SPEED]);
        int new_fan_speed = p1p2_fan_to_speed_level(payload[ControlRequest::COOLING_FAN_SPEED]);

        if (new_fan_mode != this->fan_mode)
        {
          this->fan_mode = new_fan_mode;
          state_changed = true;
        }

        // Sync fan entity speed
        if (this->fan_ != nullptr && this->fan_->speed != new_fan_speed)
        {
          this->fan_->speed = new_fan_speed;
          this->fan_->publish_state();
        }
      }

      // Handle target temperature changes
      if (this->target_temperature_updated)
      {
        if (this->mode == climate::CLIMATE_MODE_COOL)
        {
          response.cooling_setpoint = static_cast<uint8_t>(this->target_temperature);
        }
        else if (this->mode == climate::CLIMATE_MODE_HEAT)
        {
          response.heating_setpoint = static_cast<uint8_t>(this->target_temperature);
        }

        this->target_temperature_updated = false;
        state_changed = true;
      }
      else
      {
        // Read target temperature from device based on mode
        uint8_t new_target = this->target_temperature;

        if (this->mode == climate::CLIMATE_MODE_COOL)
        {
          new_target = payload[ControlRequest::COOLING_SETPOINT];
        }
        else if (this->mode == climate::CLIMATE_MODE_HEAT)
        {
          new_target = payload[ControlRequest::HEATING_SETPOINT];
        }

        if (new_target != static_cast<uint8_t>(this->target_temperature))
        {
          this->target_temperature = new_target;
          state_changed = true;
        }
      }

      // Set status flag when power is on
      if (response.power_status == PowerState::ON)
      {
        response.status_flags |= StatusFlags::POWER_ON;
      }

      // Send response
      finalize_and_send(this->homebus_, reinterpret_cast<uint8_t *>(&response), sizeof(response));

      // Publish state changes
      if (state_changed)
      {
        this->publish_state();
        this->sync_auxiliary_climate();
      }
    }

    void DaikinClimate::sync_auxiliary_climate()
    {
      if (this->aux_climate_ == nullptr)
        return;

      // Map FAN_ONLY and DRY to OFF for HomeKit compatibility
      if (this->mode == climate::CLIMATE_MODE_FAN_ONLY ||
          this->mode == climate::CLIMATE_MODE_DRY)
      {
        this->aux_climate_->mode = climate::CLIMATE_MODE_OFF;
        this->aux_climate_->action = climate::CLIMATE_ACTION_OFF;
      }
      else
      {
        this->aux_climate_->mode = this->mode;
        this->aux_climate_->action = this->action;
      }

      this->aux_climate_->target_temperature = this->target_temperature;
      this->aux_climate_->current_temperature = this->current_temperature;
      this->aux_climate_->fan_mode = this->fan_mode;
      this->aux_climate_->publish_state();
    }

    void DaikinClimate::send_simple_response(uint8_t packet_type)
    {
      uint8_t response[] = {Direction::RESPONSE, Address::AUX_CONTROLLER, packet_type, 0xFF};
      finalize_and_send(this->homebus_, response, sizeof(response));
    }

    void DaikinClimate::send_simple_response(uint8_t packet_type, const uint8_t *data, size_t data_len)
    {
      uint8_t response[16] = {Direction::RESPONSE, Address::AUX_CONTROLLER, packet_type};
      memcpy(&response[3], data, data_len);
      response[3 + data_len] = 0xFF; // CRC placeholder
      finalize_and_send(this->homebus_, response, 3 + data_len + 1);
    }

    // ==========================================================================
    // Main Callback
    // ==========================================================================

    void DaikinClimate::callback(void *arg, const uint8_t buffer[], const uint32_t buffer_length)
    {
      DaikinClimate *self = static_cast<DaikinClimate *>(arg);

      // Validate CRC
      uint8_t received_crc = buffer[buffer_length - 1];
      uint8_t calculated_crc = calculate_crc(buffer, buffer_length - 1);

      if (received_crc != calculated_crc)
      {
        ESP_LOGE(TAG, "CRC error: received 0x%02X, calculated 0x%02X (len=%lu)",
                 received_crc, calculated_crc, buffer_length);
        ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, 32, ESP_LOG_INFO);
        return;
      }

      uint8_t direction = buffer[0];
      uint8_t address = buffer[1];
      uint8_t packet_type = buffer[2];

      // Handle temperature packets (from main controller, address 0x00)
      if (address == Address::MAIN_CONTROLLER && packet_type == PacketType::TEMPERATURES)
      {
        bool is_response = (direction == Direction::RESPONSE);
        self->handle_temperature_packet(buffer, is_response);
        return;
      }

      // Handle packets addressed to us (auxiliary controller, address 0xF0)
      if (direction == Direction::REQUEST && address == Address::AUX_CONTROLLER)
      {
        const uint8_t *payload = &buffer[3];

        switch (packet_type)
        {
        case PacketType::POLL_AUX:
          ESP_LOGI(TAG, "Poll auxiliary controller");
          self->send_simple_response(PacketType::POLL_AUX);
          break;

        case PacketType::OPERATION_CONTROL:
          self->handle_control_packet(payload);
          break;

        case PacketType::COUNTER_ALARM:
        {
          ESP_LOGI(TAG, "Counter/alarm request");

          // (payload[6]) contains filter status: 0x02
          bool filter_needs_clean = (payload[6] == 0x02);
          if (self->filter_clean_binary_sensor_ != nullptr)
          {
            self->filter_clean_binary_sensor_->publish_state(filter_needs_clean);
          }

          const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00};
          self->send_simple_response(PacketType::COUNTER_ALARM, data, sizeof(data));
          break;
        }

        case PacketType::UNKNOWN_32:
        {
          ESP_LOGI(TAG, "Unknown packet 0x32");
          const uint8_t data[] = {0x01};
          self->send_simple_response(PacketType::UNKNOWN_32, data, sizeof(data));
          break;
        }

        case PacketType::UNKNOWN_3A:
        {
          ESP_LOGI(TAG, "Unknown packet 0x3A");
          const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          self->send_simple_response(PacketType::UNKNOWN_3A, data, sizeof(data));
          break;
        }

        case PacketType::OUTDOOR_UNIT_NAME:
          ESP_LOGI(TAG, "Outdoor unit name: %s", reinterpret_cast<const char *>(payload));
          self->send_simple_response(PacketType::OUTDOOR_UNIT_NAME);
          break;

        case PacketType::INDOOR_UNIT_NAME:
          ESP_LOGI(TAG, "Indoor unit name: %s", reinterpret_cast<const char *>(payload));
          self->send_simple_response(PacketType::INDOOR_UNIT_NAME);
          break;

        default:
          ESP_LOGVV(TAG, "Unhandled packet: dir=0x%02X addr=0x%02X type=0x%02X len=%lu",
                    direction, address, packet_type, buffer_length);
          break;
        }
      }
    }

    // ==========================================================================
    // DaikinClimate Implementation
    // ==========================================================================

    DaikinClimate::DaikinClimate() : climate::Climate()
    {
      this->target_temperature = DEFAULT_TARGET_TEMPERATURE;
    }

    void DaikinClimate::setup()
    {
      ESP_LOGI(TAG, "DaikinClimate::setup() starting");
      this->homebus_.register_callback(DaikinClimate::callback, this);
      this->homebus_.setup();
      ESP_LOGI(TAG, "DaikinClimate::setup() complete");
    }

    void DaikinClimate::loop()
    {
      this->homebus_.loop();
    }

    void DaikinClimate::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Daikin Ducted Climate:");
      this->homebus_.dump_config();
    }

    void DaikinClimate::control(const climate::ClimateCall &call)
    {
      if (call.get_fan_mode().has_value())
      {
        this->fan_mode = *call.get_fan_mode();
        this->fan_updated = true;
      }

      if (call.get_mode().has_value())
      {
        this->mode = *call.get_mode();
        this->mode_updated = true;
      }

      if (call.get_target_temperature().has_value())
      {
        this->target_temperature = *call.get_target_temperature();
        this->target_temperature_updated = true;
      }
    }

    climate::ClimateTraits DaikinClimate::traits()
    {
      auto traits = climate::ClimateTraits();
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
      traits.set_supported_modes({
          climate::CLIMATE_MODE_OFF,
          climate::CLIMATE_MODE_COOL,
          climate::CLIMATE_MODE_HEAT,
          climate::CLIMATE_MODE_HEAT_COOL,
          climate::CLIMATE_MODE_DRY,
          climate::CLIMATE_MODE_FAN_ONLY,
      });
      traits.set_supported_fan_modes({
          climate::CLIMATE_FAN_LOW,
          climate::CLIMATE_FAN_MEDIUM,
          climate::CLIMATE_FAN_HIGH,
      });
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
      traits.set_visual_min_temperature(MIN_TEMPERATURE);
      traits.set_visual_max_temperature(MAX_TEMPERATURE);
      traits.set_visual_target_temperature_step(TEMPERATURE_STEP);
      return traits;
    }

    // ==========================================================================
    // DaikinClimateHomeKit Implementation
    // ==========================================================================

    void DaikinClimateHomeKit::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Daikin Climate HomeKit:");
    }

    void DaikinClimateHomeKit::control(const climate::ClimateCall &call)
    {
      if (this->parent_ == nullptr)
        return;

      auto parent_call = this->parent_->make_call();

      if (call.get_mode().has_value())
        parent_call.set_mode(*call.get_mode());

      if (call.get_target_temperature().has_value())
        parent_call.set_target_temperature(*call.get_target_temperature());

      if (call.get_fan_mode().has_value())
        parent_call.set_fan_mode(*call.get_fan_mode());

      parent_call.perform();
    }

    climate::ClimateTraits DaikinClimateHomeKit::traits()
    {
      auto traits = climate::ClimateTraits();
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
      // HomeKit-compatible modes only (no FAN_ONLY or DRY)
      traits.set_supported_modes({
          climate::CLIMATE_MODE_OFF,
          climate::CLIMATE_MODE_COOL,
          climate::CLIMATE_MODE_HEAT,
          climate::CLIMATE_MODE_HEAT_COOL,
      });
      traits.set_supported_fan_modes({
          climate::CLIMATE_FAN_LOW,
          climate::CLIMATE_FAN_MEDIUM,
          climate::CLIMATE_FAN_HIGH,
      });
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
      traits.set_visual_min_temperature(MIN_TEMPERATURE);
      traits.set_visual_max_temperature(MAX_TEMPERATURE);
      traits.set_visual_target_temperature_step(TEMPERATURE_STEP);
      return traits;
    }

    // ==========================================================================
    // DaikinFan Implementation
    // ==========================================================================

    void DaikinFan::dump_config()
    {
      ESP_LOGCONFIG(TAG, "Daikin Fan:");
    }

    fan::FanTraits DaikinFan::get_traits()
    {
      auto traits = fan::FanTraits();
      traits.set_speed(true);
      traits.set_supported_speed_count(3);
      return traits;
    }

    void DaikinFan::control(const fan::FanCall &call)
    {
      if (this->parent_ == nullptr)
        return;

      auto parent_call = this->parent_->make_call();

      if (call.get_state().has_value())
      {
        bool fan_on = *call.get_state();
        parent_call.set_mode(fan_on ? climate::CLIMATE_MODE_FAN_ONLY : climate::CLIMATE_MODE_OFF);
      }

      if (call.get_speed().has_value())
      {
        int speed = *call.get_speed();
        switch (speed)
        {
        case 1:
          parent_call.set_fan_mode(climate::CLIMATE_FAN_LOW);
          break;
        case 2:
          parent_call.set_fan_mode(climate::CLIMATE_FAN_MEDIUM);
          break;
        case 3:
          parent_call.set_fan_mode(climate::CLIMATE_FAN_HIGH);
          break;
        }
      }

      parent_call.perform();
    }

  } // namespace daikin_ducted
} // namespace esphome
