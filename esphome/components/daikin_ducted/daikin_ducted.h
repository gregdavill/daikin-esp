#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "homebus_rmt.h"

namespace esphome
{
   namespace daikin_ducted
   {
      class DaikinClimate : public climate::Climate, public Component
      {
      public:
         DaikinClimate();

         void setup() override;
         void loop() override;
         void dump_config() override;
         float get_setup_priority() const override { return setup_priority::DATA; }

         void control(const climate::ClimateCall &call);
         climate::ClimateTraits traits(void);

      protected:
         static void callback(void *arg, const uint8_t buffer[], const uint32_t buffer_length);
         HomebusRMT homebus_;

         bool mode_updated = false;
         bool fan_updated = false;
         bool target_temperature_updated = false;

         float last_temp_state;

         SUB_SENSOR(indoor_temperature);
         SUB_SENSOR(outdoor_intake_temperature);
         SUB_SENSOR(coolant_temperature);

         void set_fan(fan::Fan *f) { this->fan_ = f; }
         fan::Fan *fan_{nullptr};

         void set_auxiliary_climate(climate::Climate *aux) { this->aux_climate_ = aux; }
         climate::Climate *aux_climate_{nullptr};
      };

      // HomeKit-compatible climate wrapper (only OFF/HEAT/COOL/AUTO modes)
      class DaikinClimateHomeKit : public climate::Climate, public Component
      {
      public:
         DaikinClimateHomeKit() = default;

         void setup() override {}
         void dump_config() override;
         float get_setup_priority() const override { return setup_priority::DATA - 1; }

         void control(const climate::ClimateCall &call) override;
         climate::ClimateTraits traits() override;

         void set_parent(DaikinClimate *parent) { this->parent_ = parent; }

      protected:
         DaikinClimate *parent_{nullptr};
      };

      // Fan wrapper for FAN_ONLY mode control
      class DaikinFan : public fan::Fan, public Component
      {
      public:
         DaikinFan() = default;

         void setup() override {}
         void dump_config() override;
         float get_setup_priority() const override { return setup_priority::DATA - 1; }

         fan::FanTraits get_traits() override;
         void control(const fan::FanCall &call) override;

         void set_parent(DaikinClimate *parent) { this->parent_ = parent; }

      protected:
         DaikinClimate *parent_{nullptr};
      };

   } // namespace daikin
} // namespace esphome
