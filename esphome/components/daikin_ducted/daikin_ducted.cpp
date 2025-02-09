#include "daikin_ducted.h"
#include "homebus_rmt.h"

namespace esphome
{
  namespace daikin_ducted
  {

    static const char *const TAG = "daikin_ducted.climate";

#define HOMEBUS_RX_GPIO_PIN GPIO_NUM_2
#define HOMEBUS_TX_GPIO_PIN GPIO_NUM_3

#define BUTTON_PIN GPIO_NUM_9
#define LED_PIN GPIO_NUM_4

    const unsigned char crc_table[256] = {
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

    uint8_t p1p2_crc(const uint8_t *src, unsigned int length)
    {
      uint8_t crc = 0;

      for (uint i = 0; i < length; i++)
      {
        crc = crc_table[(crc ^ *src++) & 0xFF];
      }
      return crc;
    }

    void DaikinClimate::callback(void *arg, const uint8_t buffer[], const uint32_t buffer_length)
    {
      DaikinClimate *inst = (DaikinClimate *)arg;

      // check crc
      uint8_t packet_crc = buffer[buffer_length - 1];
      uint8_t calc_crc = p1p2_crc(buffer, buffer_length - 1);

      if (packet_crc != calc_crc)
      {
        ESP_LOGE(TAG, "Packet CRC error %02x != %02x (length = %lu)", packet_crc, calc_crc, buffer_length);

        
        ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, 32, ESP_LOG_INFO);
        return;
      }

      if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x11)
      {
        int16_t temp = (buffer[8] << 8 | buffer[9]);

        ESP_LOGI(TAG, "indoor: %i.%02iC",
                 temp / 256, ((temp & 0xFF) * 100) / 256);

        float f_temperature = (float)buffer[8] + ((float)buffer[9] / 256.f);
        inst->current_temperature = f_temperature;

        if(fabs(inst->last_temp_state - inst->current_temperature) > 0.08f){
          inst->publish_state();
          inst->last_temp_state = inst->current_temperature;
        }

        if (inst->indoor_temperature_sensor_ != nullptr)
        {
          inst->indoor_temperature_sensor_->publish_state(f_temperature);
        }
      }

      if (buffer[0] == 0x40 && buffer[1] == 0x00 && buffer[2] == 0x11)
      {
        int16_t temp = (buffer[8] << 8 | buffer[9]);

        ESP_LOGI(TAG, "intake: %i.%02iC",
                 temp / 256, ((temp & 0xFF) * 100) / 256);

        float f_temperature = (float)buffer[8] + ((float)buffer[9] / 256.f);
        if (inst->outdoor_intake_temperature_sensor_ != nullptr)
        {
          inst->outdoor_intake_temperature_sensor_->publish_state(f_temperature);
        }

        float f_temperature0 = (float)buffer[14] + ((float)buffer[15] / 256.f);
        if (inst->coolant_temperature_sensor_ != nullptr)
        {
          inst->coolant_temperature_sensor_->publish_state(f_temperature0);
        }
      }


      /* Targeting us */
      if (buffer[0] == 0x00 && buffer[1] == 0xF0)
      {

        const uint8_t *payload = &buffer[3];

        switch (buffer[2])
        {
        case 0x30:
        {
          ESP_LOGI(TAG, "Peripheral Ping, responding.");
          uint8_t ping_response[] = {0x40, 0xF0, 0x30,
                                     0xFF};

          ping_response[sizeof(ping_response) - 1] = p1p2_crc(ping_response, sizeof(ping_response) - 1);
          inst->homebus_.write_bytes(ping_response, sizeof(ping_response));

          break;
        }

        case 0x38:
        {
          ESP_LOGI(TAG, "FXMQ control message (0x38)");

          static int i = 0;
          bool new_params = false;

          uint8_t ping_response[] = {
              0x40, 0xf0, 0x38,             // header
              (uint8_t)(payload[0] & 0x01), // target status
              payload[2],                   // target operating mode
              payload[4],                   // target temperature_cooling
              0x00,
              payload[6], // target fan_speed
              0x00,
              payload[8], // target temperature_heating
              0x00,
              payload[10], // target heat_fan speed
              payload[11], // unknown
              0x00,
              0x00,
              0x00,
              payload[15], // C0, E0 when payload[0] set to 1
              0x00,
              0x00,
              0x00,
              0xFF // crc byte
          };

          if (inst->mode_updated)
          {
            /* Set device from HA control */
            bool pwr_on = false;
            if (inst->mode != climate::CLIMATE_MODE_OFF)
            {
              pwr_on = true;
            }

            ping_response[3] = pwr_on;

            if (inst->mode == climate::CLIMATE_MODE_FAN_ONLY)
              ping_response[4] = 0x60;
            if (inst->mode == climate::CLIMATE_MODE_HEAT)
              ping_response[4] = 0x61;
            if (inst->mode == climate::CLIMATE_MODE_COOL)
              ping_response[4] = 0x62;
            if (inst->mode == climate::CLIMATE_MODE_AUTO)
              ping_response[4] = 0x63;
            if (inst->mode == climate::CLIMATE_MODE_DRY)
              ping_response[4] = 0x67;

            ping_response[6] = 0x80;

            inst->mode_updated = false;
            new_params = true;
          }
          else
          {
            /* Set HA from device control */
            auto new_mode = inst->mode;
            if (payload[0] == 0)
              new_mode = climate::CLIMATE_MODE_OFF;
            else if (payload[2] == 0x60)
              new_mode = climate::CLIMATE_MODE_FAN_ONLY;
            else if (payload[2] == 0x61)
              new_mode = climate::CLIMATE_MODE_HEAT;
            else if (payload[2] == 0x62)
              new_mode = climate::CLIMATE_MODE_COOL;
            else if (payload[2] == 0x63)
              new_mode = climate::CLIMATE_MODE_AUTO;
            else if (payload[2] == 0x67)
              new_mode = climate::CLIMATE_MODE_DRY;

            if (new_mode != inst->mode)
              new_params = true;
            inst->mode = new_mode;
          }

          {
            /* Set HA from device control */
            auto new_action = inst->action;
            if (payload[0] == 0)
              new_action = climate::CLIMATE_ACTION_OFF;
            else if (payload[3] == 0x00)
              new_action = climate::CLIMATE_ACTION_FAN;
            else if (payload[3] == 0x01)
              new_action = climate::CLIMATE_ACTION_HEATING;
            else if (payload[3] == 0x02)
              new_action = climate::CLIMATE_ACTION_COOLING;

            if (new_action != inst->action)
              new_params = true;
            inst->action = new_action;
          }

          if (inst->fan_updated)
          {
            uint8_t fan_mode = 0;
            if (inst->fan_mode == climate::CLIMATE_FAN_LOW)
              fan_mode = 0x10;
            else if (inst->fan_mode == climate::CLIMATE_FAN_MEDIUM)
              fan_mode = 0x30;
            else if (inst->fan_mode == climate::CLIMATE_FAN_HIGH)
              fan_mode = 0x50;

            ping_response[7] = (payload[6] & ~0x70) | fan_mode | 0x80;
            ping_response[11] = (payload[10] & ~0x70) | fan_mode | 0x80;
            inst->fan_updated = false;
            new_params = true;
          }
          else
          {
            auto new_fan_mode = inst->fan_mode;
            if ((payload[6] & 0x70) == 0x10)
              new_fan_mode = climate::CLIMATE_FAN_LOW;
            if ((payload[6] & 0x70) == 0x30)
              new_fan_mode = climate::CLIMATE_FAN_MEDIUM;
            if ((payload[6] & 0x70) == 0x50)
              new_fan_mode = climate::CLIMATE_FAN_HIGH;

            if (new_fan_mode != inst->fan_mode)
              new_params = true;
            inst->fan_mode = new_fan_mode;
          }

          if (inst->target_temperature_updated)
          {
            if (inst->mode == climate::CLIMATE_MODE_COOL)
              ping_response[5] = inst->target_temperature;
            if (inst->mode == climate::CLIMATE_MODE_HEAT)
              ping_response[9] = inst->target_temperature;

            inst->target_temperature_updated = false;
            new_params = true;
          }
          else
          {
            uint8_t new_target_temperature = inst->target_temperature;

            if (inst->mode == climate::CLIMATE_MODE_COOL)
              new_target_temperature = payload[4];
            if (inst->mode == climate::CLIMATE_MODE_HEAT)
              new_target_temperature = payload[8];

            if (new_target_temperature != inst->target_temperature)
              new_params = true;
            inst->target_temperature = new_target_temperature;
          }

          /* Transmit control packet to device */

          if (ping_response[3])
          {
            ping_response[16] |= 0x20;
          }

          ping_response[sizeof(ping_response) - 1] = p1p2_crc(ping_response, sizeof(ping_response) - 1);
          inst->homebus_.write_bytes(ping_response, sizeof(ping_response));

          if (new_params)
          {
            new_params = false;
            inst->publish_state();
          }
          break;
        }

        case 0x39:
        {
          ESP_LOGI(TAG, "Peripheral counter_alarm");

          uint8_t ping_response[] = {0x40, 0xF0, 0x39,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
          ping_response[sizeof(ping_response) - 1] = p1p2_crc(ping_response, sizeof(ping_response) - 1);

          inst->homebus_.write_bytes(ping_response, sizeof(ping_response));
          break;
        }

        case 0x32:
        {
          ESP_LOGI(TAG, "Peripheral (0x32)");

          uint8_t ping_response[] = {0x40, 0xF0, 0x32,
                                     0x01, 0xFF};
          ping_response[sizeof(ping_response) - 1] = p1p2_crc(ping_response, sizeof(ping_response) - 1);

          inst->homebus_.write_bytes(ping_response, sizeof(ping_response));
          break;
        }

        case 0x3a:
        {
          ESP_LOGI(TAG, "Peripheral (0x3A)");

          uint8_t ping_response[] = {0x40, 0xF0, 0x3a,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF};
          ping_response[sizeof(ping_response) - 1] = p1p2_crc(ping_response, sizeof(ping_response) - 1);

          inst->homebus_.write_bytes(ping_response, sizeof(ping_response));
          break;
        }

        case 0x35:
        {
          ESP_LOGI(TAG, "Outside Name(0x35): %s", &buffer[3]);

          uint8_t ping_response[] = {0x40, 0xF0, 0x35,
                                     0xFF};
          ping_response[sizeof(ping_response) - 1] = p1p2_crc(ping_response, sizeof(ping_response) - 1);

          inst->homebus_.write_bytes(ping_response, sizeof(ping_response));
          break;
        }
        case 0x36:
        {
          ESP_LOGI(TAG, "Inside Name(0x36): %s", &buffer[3]);

          uint8_t ping_response[] = {0x40, 0xF0, 0x36,
                                     0xFF};
          ping_response[sizeof(ping_response) - 1] = p1p2_crc(ping_response, sizeof(ping_response) - 1);

          inst->homebus_.write_bytes(ping_response, sizeof(ping_response));
          break;
        }

        default:
          ESP_LOGVV(TAG, "header=%02x%02x%02x (size=%lu)", buffer[0], buffer[1], buffer[2], buffer_length);
          return;
        }
      }
    }

    DaikinClimate::DaikinClimate()
        : climate::Climate()
    {
      this->target_temperature = 25;

      this->homebus_.register_callback(DaikinClimate::callback, this);
      this->homebus_.setup();
      ESP_LOGI(TAG, "homebus installed");
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
      traits.set_supports_current_temperature(true);
      traits.set_supported_modes({climate::CLIMATE_MODE_OFF,
                                  climate::CLIMATE_MODE_COOL,
                                  climate::CLIMATE_MODE_HEAT,
                                  climate::CLIMATE_MODE_HEAT_COOL,
                                  climate::CLIMATE_MODE_DRY,
                                  climate::CLIMATE_MODE_FAN_ONLY});
      traits.set_supported_fan_modes({climate::CLIMATE_FAN_LOW,
                                      climate::CLIMATE_FAN_MEDIUM,
                                      climate::CLIMATE_FAN_HIGH});
      traits.set_supports_action(true);
      traits.set_visual_min_temperature(18.0);
      traits.set_visual_max_temperature(28.0);
      traits.set_visual_target_temperature_step(1.0);
      return traits;
    }

  } // namespace daikin_ducted
} // namespace esphome
