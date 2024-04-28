
#include <stdint.h>

typedef struct {
    uint8_t direction;
    uint8_t peripheral_adr;
    uint8_t type;
    uint8_t payload[];
} p1p2_packet_t;

enum direction {
    DIR_REQUEST = 0x00,
    DIR_RESPONSE = 0x40,
    DIR_INIT = 0x80
};


/* Packet Type 0x10 (OPERATING_STATUS) */
typedef struct {
    uint8_t power; //0
    uint8_t target_operating_mode; //1
    uint8_t actual_operating_mode; //2
    uint8_t target_temperature; //3
    uint8_t unknown4; //4
    uint8_t fan_speed; //5
    uint8_t unknown6; //6
    uint8_t unknown7; //7
    uint8_t unknown8; //8
    uint8_t unknown9; //9
    uint8_t unknown10; //10
    uint8_t unknown11; //11
    uint8_t unknown12; //12
    uint8_t compressor_state; //13
    uint8_t unknown14; //14
    uint8_t unknown15; //15
    uint8_t unknown16; //16
    uint8_t unknown17; //17
    uint8_t unknown18; //18
    uint8_t unknown19; //19
} p1p2_OperatingStatus_t;

enum TargetOperatingMode {
    FAN = 0x60,
    HEAT = 0x61,
    COOL = 0x62,
    AUTO = 0x63,
    DRY = 0x67,
};

// enum ActualOperatingMode {
//     FAN = 0x00,
//     HEAT = 0x01,
//     COOL = 0x02,
// };

// enum FanSpeed {
//     FAN = 0x00,
//     HEAT = 0x01,
//     COOL = 0x02,
// };


/* Packet Type 0x11 (TEMPERATURES) */
typedef struct {
    uint8_t unknown0; //0
    uint8_t unknown1; //1
    uint8_t unknown2; //2
    uint8_t unknown3; //3
    uint8_t unknown4; //4
    int8_t temp_int; //5
    int8_t temp_fraction; //6
    uint8_t unknown7; //7
    uint8_t unknown8; //8
    uint8_t unknown9; //9
} p1p2_Temperatures_t;

/* Packet Type 0x30 (POLL_AUX_CONTROLLER) */
typedef struct {
    uint8_t unknown0; //0
    uint8_t unknown1; //1
    uint8_t unknown2; //2
    uint8_t unknown3; //3
    uint8_t unknown4; //4
    uint8_t unknown5; //5
    uint8_t unknown6; //6
    uint8_t unknown7; //7
    uint8_t unknown8; //8
    uint8_t unknown9; //9
    uint8_t unknown10; //10
    uint8_t unknown11; //11
    uint8_t unknown12; //12
    uint8_t unknown13; //13
    uint8_t unknown14; //14
    uint8_t unknown15; //15
    uint8_t unknown16; //16
    uint8_t unknown17; //17
    uint8_t unknown18; //18
    uint8_t unknown19; //19
} p1p2_PollAuxController_t;


/* Packet Type 0x38 (OPERATION CONTROL) */
typedef struct {
    uint8_t unknown0; //0
    uint8_t unknown1; //1
    uint8_t unknown2; //2
    uint8_t unknown3; //3
    uint8_t unknown4; //4
    uint8_t unknown5; //5
    uint8_t unknown6; //6
    uint8_t unknown7; //7
    uint8_t unknown8; //8
    uint8_t unknown9; //9
    uint8_t unknown10; //10
    uint8_t unknown11; //11
    uint8_t unknown12; //12
    uint8_t unknown13; //13
    uint8_t unknown14; //14
    uint8_t unknown15; //15
} p1p2_OperatingControl_t;


void p1p2_parse_packet(const unsigned int *buffer, unsigned int buffer_length);
