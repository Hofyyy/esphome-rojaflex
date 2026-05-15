import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover, text_sensor
from esphome.const import CONF_ID, CONF_CHANNEL

from .. import CONF_ROJAFLEX_ID, RojaflexHub, rojaflex_ns

DEPENDENCIES = ["rojaflex"]

RojaflexCover = rojaflex_ns.class_("RojaflexCover", cover.Cover, cg.Component)

CONF_STATUS_SENSOR = "status_sensor"

# Schema for the optional per-channel status text sensor
STATUS_SENSOR_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
    }
)

CONFIG_SCHEMA = cover.cover_schema(RojaflexCover).extend(
    {
        cv.Required(CONF_ROJAFLEX_ID): cv.use_id(RojaflexHub),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=15),
        cv.Optional(CONF_STATUS_SENSOR): STATUS_SENSOR_SCHEMA,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    hub = await cg.get_variable(config[CONF_ROJAFLEX_ID])
    cg.add(var.set_rojaflex_parent(hub))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
    cg.add(hub.register_cover(var))

    if CONF_STATUS_SENSOR in config:
        ts = cg.new_Pvariable(config[CONF_STATUS_SENSOR][CONF_ID])
        await text_sensor.register_text_sensor(ts, config[CONF_STATUS_SENSOR])
        cg.add(var.set_status_sensor(ts))
