import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover, uart
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_USE_ADDRESS

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["cover"]

bus_t4_ns = cg.esphome_ns.namespace("bus_t4")
Nice = bus_t4_ns.class_("NiceBusT4", cover.Cover, cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    cover.COVER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(Nice),
            cv.Optional(CONF_ADDRESS): cv.hex_uint16_t,
            cv.Optional(CONF_USE_ADDRESS): cv.hex_uint16_t,
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await cover.register_cover(var, config)

    if CONF_ADDRESS in config:
        cg.add(var.set_to_address(config[CONF_ADDRESS]))

    if CONF_USE_ADDRESS in config:
        cg.add(var.set_from_address(config[CONF_USE_ADDRESS]))

