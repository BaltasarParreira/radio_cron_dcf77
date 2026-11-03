import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import time, switch
from esphome.const import CONF_ID, CONF_TIME_ID

DEPENDENCIES = ["time"]
MULTI_CONF = True

idf_dcf77_emitter_ns = cg.esphome_ns.namespace("idf_dcf77_emitter")

IDFDCF77Emitter = idf_dcf77_emitter_ns.class_(
    "IDFDCF77Emitter",
    cg.Component
)

CONF_ANTENNA_PIN = "antenna_pin"
CONF_LED_PIN = "led_pin"
CONF_SYNC_SWITCH_ID = "sync_switch_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(IDFDCF77Emitter),

    cv.Required(CONF_TIME_ID):
        cv.use_id(time.RealTimeClock),

    cv.Required(CONF_ANTENNA_PIN):
        pins.gpio_output_pin_schema,

    cv.Required(CONF_LED_PIN):
        pins.gpio_output_pin_schema,

    cv.Required(CONF_SYNC_SWITCH_ID):
        cv.use_id(switch.Switch),

}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    time_ = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time_id(time_))

    antenna_pin = await cg.gpio_pin_expression(
        config[CONF_ANTENNA_PIN]
    )
    cg.add(var.set_antenna_pin(antenna_pin))

    led_pin = await cg.gpio_pin_expression(
        config[CONF_LED_PIN]
    )
    cg.add(var.set_led_pin(led_pin))

    sync_switch = await cg.get_variable(
        config[CONF_SYNC_SWITCH_ID]
    )
    cg.add(var.set_sync_switch(sync_switch))
