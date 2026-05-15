import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@Hofyyy"]
DEPENDENCIES = ["cc1101"]
AUTO_LOAD = ["text_sensor"]

rojaflex_ns = cg.esphome_ns.namespace("rojaflex")
RojaflexHub = rojaflex_ns.class_("RojaflexHub", cg.Component)

# Public constants used by sub-platforms
CONF_ROJAFLEX_ID = "rojaflex_id"
CONF_CC1101_ID = "cc1101_id"
CONF_HOUSECODE = "housecode"
CONF_TX_REPETITIONS = "tx_repetitions"

# CC1101 component class reference (namespace access avoids import-cycle)
_cc1101_ns = cg.esphome_ns.namespace("cc1101")
CC1101Component = _cc1101_ns.class_("CC1101Component")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(RojaflexHub),
        cv.Required(CONF_CC1101_ID): cv.use_id(CC1101Component),
        cv.Optional(CONF_HOUSECODE, default="0000000"): cv.string,
        cv.Optional(CONF_TX_REPETITIONS, default=2): cv.int_range(min=1, max=4),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cc1101_component = await cg.get_variable(config[CONF_CC1101_ID])
    cg.add(var.set_cc1101_parent(cc1101_component))
    cg.add(var.set_housecode(config[CONF_HOUSECODE]))
    cg.add(var.set_tx_repetitions(config[CONF_TX_REPETITIONS]))
