import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_DISABLED_BY_DEFAULT, CONF_ID, CONF_TYPE

from .. import CONF_ROJAFLEX_ID, RojaflexHub

DEPENDENCIES = ["rojaflex"]

# Supported types and the hub setter they wire to.
# "housecode" doubles as setup-state display ("Receiving..." / "Learning: 0xXXX (n/N)"
# / "0xXXXXXXX"). "last_tx" publishes "ok" or short CC1101Error name on failure.
_SENSOR_TYPES = {
    "housecode": "set_housecode_sensor",
    "last_rx_info": "set_last_rx_info_sensor",
    "last_tx": "set_last_tx_sensor",
}

# Debug-only types: disabled by default in HA so they don't bloat the recorder
# database with hex/raw frame dumps. User can re-enable any single entity from
# the HA device page, or override per-entity via `disabled_by_default: false`.
_DEFAULT_DISABLED_TYPES = {"last_rx_info"}

_BASE_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
        cv.Required(CONF_ROJAFLEX_ID): cv.use_id(RojaflexHub),
        cv.Required(CONF_TYPE): cv.one_of(*_SENSOR_TYPES, lower=True),
    }
)


def CONFIG_SCHEMA(value):  # noqa: N802 — ESPHome convention
    # In ESPHome 2026.x `disabled_by_default` is read from `config` BEFORE the
    # codegen runs (entity_helpers.py packs all flags into a single C++ call),
    # so we have to mutate the config — there is no C++ setter. The base
    # schema applies a default of `False`, which is indistinguishable from a
    # user-set `False`. Sniff the raw input first so we only apply our
    # per-type default when the user didn't explicitly set the field.
    user_set_dbd = isinstance(value, dict) and CONF_DISABLED_BY_DEFAULT in value
    config = _BASE_SCHEMA(value)
    if not user_set_dbd:
        config[CONF_DISABLED_BY_DEFAULT] = config[CONF_TYPE] in _DEFAULT_DISABLED_TYPES
    return config


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await text_sensor.register_text_sensor(var, config)

    hub = await cg.get_variable(config[CONF_ROJAFLEX_ID])
    cg.add(getattr(hub, _SENSOR_TYPES[config[CONF_TYPE]])(var))
