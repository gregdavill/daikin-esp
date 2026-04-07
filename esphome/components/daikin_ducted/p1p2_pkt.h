#pragma once

#include <stdint.h>

namespace esphome
{
namespace daikin_ducted
{

// =============================================================================
// P1P2 Protocol Constants
// =============================================================================

// Packet direction (first byte)
namespace Direction
{
  constexpr uint8_t REQUEST = 0x00;
  constexpr uint8_t RESPONSE = 0x40;
  constexpr uint8_t INIT = 0x80;
}

// Device addresses (second byte)
namespace Address
{
  constexpr uint8_t MAIN_CONTROLLER = 0x00;
  constexpr uint8_t AUX_CONTROLLER = 0xF0;
}

// Packet types (third byte)
namespace PacketType
{
  constexpr uint8_t OPERATING_STATUS = 0x10;
  constexpr uint8_t TEMPERATURES = 0x11;
  constexpr uint8_t POLL_AUX = 0x30;
  constexpr uint8_t UNKNOWN_32 = 0x32;
  constexpr uint8_t OUTDOOR_UNIT_NAME = 0x35;
  constexpr uint8_t INDOOR_UNIT_NAME = 0x36;
  constexpr uint8_t OPERATION_CONTROL = 0x38;
  constexpr uint8_t COUNTER_ALARM = 0x39;
  constexpr uint8_t UNKNOWN_3A = 0x3A;
  constexpr uint8_t UNKNOWN_31 = 0x31;
}

// Operating modes
namespace OperatingMode
{
  constexpr uint8_t FAN_ONLY = 0x60;
  constexpr uint8_t HEAT = 0x61;
  constexpr uint8_t COOL = 0x62;
  constexpr uint8_t AUTO = 0x63;
  constexpr uint8_t DRY = 0x67;
}

// Current action/compressor state
namespace ActionState
{
  constexpr uint8_t IDLE = 0x00;      // Fan running, compressor off
  constexpr uint8_t HEATING = 0x01;
  constexpr uint8_t COOLING = 0x02;
}

// Fan speed values (bits 4-6, masked with 0x70)
namespace FanSpeed
{
  constexpr uint8_t LOW = 0x10;
  constexpr uint8_t MEDIUM = 0x30;
  constexpr uint8_t HIGH = 0x50;
  constexpr uint8_t MASK = 0x70;
  constexpr uint8_t CHANGED_FLAG = 0x80;
}

// Power state
namespace PowerState
{
  constexpr uint8_t OFF = 0x00;
  constexpr uint8_t ON = 0x01;
}

// =============================================================================
// Packet Structures
// =============================================================================

// Generic P1P2 packet header
struct PacketHeader
{
  uint8_t direction;
  uint8_t address;
  uint8_t type;
};

// Packet Type 0x11 - Temperature readings
// Request (direction=0x00): indoor temperature at offset 8-9
// Response (direction=0x40): outdoor intake at 8-9, coolant at 14-15
namespace TemperaturePacket
{
  constexpr size_t INDOOR_TEMP_OFFSET = 8;
  constexpr size_t OUTDOOR_INTAKE_TEMP_OFFSET = 8;
  constexpr size_t COOLANT_TEMP_OFFSET = 14;
}

// Packet Type 0x38 - Operation Control (Request payload offsets)
namespace ControlRequest
{
  constexpr size_t POWER_STATUS = 0;
  constexpr size_t OPERATING_MODE = 2;
  constexpr size_t CURRENT_ACTION = 3;
  constexpr size_t COOLING_SETPOINT = 4;
  constexpr size_t COOLING_FAN_SPEED = 6;
  constexpr size_t HEATING_SETPOINT = 8;
  constexpr size_t HEATING_FAN_SPEED = 10;
  constexpr size_t UNKNOWN_11 = 11;
  constexpr size_t STATUS_FLAGS = 15;
}

// Packet Type 0x38 - Operation Control (Response structure)
struct ControlResponse
{
  uint8_t header[3];           // 0-2: direction, address, type
  uint8_t power_status;        // 3: 0=off, 1=on
  uint8_t operating_mode;      // 4: see OperatingMode
  uint8_t cooling_setpoint;    // 5: target temp for cooling
  uint8_t reserved_6;          // 6: usually 0x00
  uint8_t cooling_fan_speed;   // 7: fan speed + flags
  uint8_t reserved_8;          // 8: usually 0x00
  uint8_t heating_setpoint;    // 9: target temp for heating
  uint8_t reserved_10;         // 10: usually 0x00
  uint8_t heating_fan_speed;   // 11: fan speed + flags
  uint8_t unknown_12;          // 12: copied from request
  uint8_t reserved_13;         // 13: usually 0x00
  uint8_t reserved_14;         // 14: usually 0x00
  uint8_t reserved_15;         // 15: usually 0x00
  uint8_t status_flags;        // 16: 0x20 when power on
  uint8_t reserved_17;         // 17: usually 0x00
  uint8_t reserved_18;         // 18: usually 0x00
  uint8_t reserved_19;         // 19: usually 0x00
  uint8_t crc;                 // 20: CRC byte
};

namespace StatusFlags
{
  constexpr uint8_t POWER_ON = 0x20;
}

// =============================================================================
// Helper Functions
// =============================================================================

// Convert fixed-point temperature (integer + fraction/256) to float
inline float fixed_point_to_float(uint8_t integer, uint8_t fraction)
{
  return static_cast<float>(integer) + (static_cast<float>(fraction) / 256.0f);
}

// Temperature change threshold for publishing state updates
constexpr float TEMP_PUBLISH_THRESHOLD = 0.08f;

// Default target temperature
constexpr uint8_t DEFAULT_TARGET_TEMPERATURE = 25;

// Temperature limits
constexpr float MIN_TEMPERATURE = 18.0f;
constexpr float MAX_TEMPERATURE = 28.0f;
constexpr float TEMPERATURE_STEP = 1.0f;

} // namespace daikin_ducted
} // namespace esphome
