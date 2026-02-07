import logging
from esphome import git

from esphome.components import text_sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

_LOGGER = logging.getLogger(__name__)

ICON_SOURCE_BRANCH = "mdi:source-branch"

git_ref_ns = cg.esphome_ns.namespace("git_ref")
git_ref_TextSensor = git_ref_ns.class_(
    "GitRefTextSensor", text_sensor.TextSensor, cg.PollingComponent
)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(
        git_ref_TextSensor,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon=ICON_SOURCE_BRANCH,
    )
    .extend(
        {
            cv.Optional("commit-ish"): cv.string,
            cv.Optional("broken"): cv.string,
            cv.Optional("dirty"): cv.string,
            cv.Optional("unversioned", default="-UNVERSIONED"): cv.string,
            cv.Optional("all", default=False): cv.boolean,
            cv.Optional("tags", default=False): cv.boolean,
            cv.Optional("long", default=False): cv.boolean,
            cv.Optional("always", default=False): cv.boolean,
            cv.Optional("abbrev"): cv.positive_int,
        }
    )
    .extend(cv.polling_component_schema("never"))
)

def produce_git_describe(config):
    dirty_postfix = ""

    GIT_ROOT_DIR = git.run_git_command(
        ["git", "rev-parse"]
    )

    COMMAND = ["git", "describe"]

    if config.get("all", False):
        COMMAND.append("--all")
    if config.get("tags", False):
        COMMAND.append("--tags")
    if config.get("long", False):
        COMMAND.append("--long")
    if config.get("always", False):
        COMMAND.append("--always")

    if "abbrev" in config:
        COMMAND.append(f"--abbrev={config['abbrev']}")
    if "broken" in config:
        COMMAND.append(f"--broken={config['broken']}")
    if "commit-ish" in config:
        COMMAND.append(f"{config['commit-ish']}")

    _LOGGER.info("  Command: %s", COMMAND)
    _describe = git.run_git_command(COMMAND)
    _describe += dirty_postfix
    _LOGGER.info("  GIT Describe result: %s", _describe)

    return _describe


async def to_code(config):
    git_ref_result = produce_git_describe(config)
    var = cg.new_Pvariable(config[CONF_ID], git_ref_result)
    await cg.register_component(var, config)
    await text_sensor.register_text_sensor(var, config)
