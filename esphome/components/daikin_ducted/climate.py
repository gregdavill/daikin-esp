import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, climate, fan, sensor
from esphome.const import (
    CONF_ID,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_PROBLEM,
    STATE_CLASS_MEASUREMENT,
)

CONF_HOMEKIT_CLIMATE = "homekit_climate"
CONF_HOMEKIT_FAN = "homekit_fan"
CONF_FILTER_CLEAN = "filter_clean"

AUTO_LOAD = ["binary_sensor", "climate", "fan", "sensor"]

daikin_ns = cg.esphome_ns.namespace("daikin_ducted")
DaikinClimate = daikin_ns.class_("DaikinClimate", climate.Climate, cg.Component)
DaikinClimateHomeKit = daikin_ns.class_("DaikinClimateHomeKit", climate.Climate, cg.Component)
DaikinFan = daikin_ns.class_("DaikinFan", fan.Fan, cg.Component)

CONFIG_SCHEMA = climate.climate_schema(DaikinClimate).extend(
    {
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
        cv.Optional(CONF_HOMEKIT_CLIMATE): climate.climate_schema(DaikinClimateHomeKit),
        cv.Optional(CONF_HOMEKIT_FAN): fan.fan_schema(DaikinFan),
        cv.Optional(CONF_FILTER_CLEAN): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
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
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    for key, funcName in SENSOR_TYPES.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, funcName)(sens))

    if CONF_FILTER_CLEAN in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_FILTER_CLEAN])
        cg.add(var.set_filter_clean_binary_sensor(sens))

    if CONF_HOMEKIT_CLIMATE in config:
        hk_config = config[CONF_HOMEKIT_CLIMATE]
        hk_var = cg.new_Pvariable(hk_config[CONF_ID])
        await cg.register_component(hk_var, hk_config)
        await climate.register_climate(hk_var, hk_config)
        cg.add(hk_var.set_parent(var))
        cg.add(var.set_auxiliary_climate(hk_var))

    if CONF_HOMEKIT_FAN in config:
        fan_config = config[CONF_HOMEKIT_FAN]
        fan_var = cg.new_Pvariable(fan_config[CONF_ID])
        await cg.register_component(fan_var, fan_config)
        await fan.register_fan(fan_var, fan_config)
        cg.add(fan_var.set_parent(var))
        cg.add(var.set_fan(fan_var))
