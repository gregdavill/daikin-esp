import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID

AUTO_LOAD = ["climate"]

daikin_ns = cg.esphome_ns.namespace("daikin_ducted")
DaikinClimate = daikin_ns.class_("DaikinClimate", climate.Climate)

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(DaikinClimate),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await climate.register_climate(var, config)
