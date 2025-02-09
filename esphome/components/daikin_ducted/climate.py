import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID

from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
)


AUTO_LOAD = ["climate", "sensor"]

daikin_ns = cg.esphome_ns.namespace("daikin_ducted")
DaikinClimate = daikin_ns.class_("DaikinClimate", climate.Climate)

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(DaikinClimate),
        cv.Optional("outdoor_intake_temperature"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("coolant_temperature"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon="mdi:coolant-temperature",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("indoor_temperature"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon="mdi:home-thermometer",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

SENSOR_TYPES = {
    "outdoor_intake_temperature": "set_outdoor_intake_temperature_sensor",
    "coolant_temperature": "set_coolant_temperature_sensor",
    "indoor_temperature": "set_indoor_temperature_sensor",
}

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await climate.register_climate(var, config)

    for key, funcName in SENSOR_TYPES.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, funcName)(sens))
