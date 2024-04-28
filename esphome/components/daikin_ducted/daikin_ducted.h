#pragma once

#include "esphome/components/climate/climate.h"
#include "homebus_rmt.h"

namespace esphome
{
   namespace daikin_ducted
   {
      class DaikinClimate : public climate::Climate
      {
      public:
         DaikinClimate();

         void control(const climate::ClimateCall &call);
         climate::ClimateTraits traits(void);

      protected:
         static void callback(void *arg, const uint8_t buffer[], const uint32_t buffer_length);
         HomebusRMT homebus_;

         bool mode_updated = false;
         bool fan_updated = false;
         bool target_temperature_updated = false;

         float last_temp_state;
      };

   } // namespace daikin
} // namespace esphome
