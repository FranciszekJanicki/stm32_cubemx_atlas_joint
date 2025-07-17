#include "as5600.h"
#include <assert.h>
#include <string.h>

static as5600_err_t as5600_bus_initialize(as5600_t const* as5600)
{
    return as5600->interface.bus_initialize
               ? as5600->interface.bus_initialize(as5600->interface.bus_user)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_bus_deinitialize(as5600_t const* as5600)
{
    return as5600->interface.bus_deinitialize
               ? as5600->interface.bus_deinitialize(as5600->interface.bus_user)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_bus_write_data(as5600_t const* as5600,
                                          uint8_t address,
                                          uint8_t const* data,
                                          size_t data_size)
{
    return as5600->interface.bus_write_data
               ? as5600->interface.bus_write_data(as5600->interface.bus_user,
                                                  address,
                                                  data,
                                                  data_size)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_bus_read_data(as5600_t const* as5600,
                                         uint8_t address,
                                         uint8_t* data,
                                         size_t data_size)
{
    return as5600->interface.bus_read_data
               ? as5600->interface.bus_read_data(as5600->interface.bus_user,
                                                 address,
                                                 data,
                                                 data_size)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_adc_initialize(as5600_t const* as5600)
{
    return as5600->interface.adc_initialize
               ? as5600->interface.adc_initialize(as5600->interface.adc_user)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_adc_deinitialize(as5600_t const* as5600)
{
    return as5600->interface.adc_deinitialize
               ? as5600->interface.adc_deinitialize(as5600->interface.adc_user)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_adc_get_conversion(as5600_t const* as5600, uint16_t* conversion)
{
    return as5600->interface.adc_get_conversion
               ? as5600->interface.adc_get_conversion(as5600->interface.adc_user, conversion)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_gpio_initialize(as5600_t const* as5600)
{
    return as5600->interface.gpio_initialize
               ? as5600->interface.gpio_initialize(as5600->interface.gpio_user)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_gpio_deinitialize(as5600_t const* as5600)
{
    return as5600->interface.gpio_deinitialize
               ? as5600->interface.gpio_deinitialize(as5600->interface.gpio_user)
               : AS5600_ERR_NULL;
}

static as5600_err_t as5600_gpio_write_pin(as5600_t const* as5600, uint32_t pin, bool state)
{
    return as5600->interface.gpio_write_pin
               ? as5600->interface.gpio_write_pin(as5600->interface.gpio_user, pin, state)
               : AS5600_ERR_NULL;
}

as5600_err_t as5600_initialize(as5600_t* as5600,
                               as5600_config_t const* config,
                               as5600_interface_t const* interface)
{
    assert(as5600 && config && interface);

    memset(as5600, 0, sizeof(*as5600));
    memcpy(&as5600->config, config, sizeof(*config));
    memcpy(&as5600->interface, interface, sizeof(*interface));

    as5600_err_t err = as5600_bus_initialize(as5600);
    err |= as5600_adc_initialize(as5600);
    err |= as5600_gpio_initialize(as5600);

    return err;
}

as5600_err_t as5600_deinitialize(as5600_t* as5600)
{
    assert(as5600);

    as5600_err_t err = as5600_bus_deinitialize(as5600);
    err |= as5600_adc_deinitialize(as5600);
    err |= as5600_gpio_deinitialize(as5600);

    memset(as5600, 0, sizeof(*as5600));

    return err;
}

as5600_err_t as5600_set_direction(as5600_t const* as5600, as5600_direction_t direction)
{
    assert(as5600);

    return as5600_gpio_write_pin(as5600, as5600->config.dir_pin, direction & 0x01U);
}

as5600_err_t as5600_get_angle_data_scaled_adc(as5600_t const* as5600, float32_t* scaled)
{
    assert(as5600 && scaled);

    uint16_t raw = {};

    as5600_err_t err = as5600_get_angle_data_raw_adc(as5600, &raw);

    *scaled = (float32_t)raw * (as5600->config.max_angle - as5600->config.min_angle) / 4096.0F +
              as5600->config.min_angle;

    return err;
}

as5600_err_t as5600_get_angle_data_raw_adc(as5600_t const* as5600, uint16_t* raw)
{
    assert(as5600 && raw);

    return as5600_adc_get_conversion(as5600, raw);
}

as5600_err_t as5600_get_angle_data_scaled_bus(as5600_t const* as5600, float32_t* scaled)
{
    assert(as5600 && scaled);

    uint16_t raw = {};

    as5600_err_t err = as5600_get_angle_data_raw_bus(as5600, &raw);

    *scaled = (float32_t)raw * (as5600->config.max_angle - as5600->config.min_angle) / 4096.0F +
              as5600->config.min_angle;

    return err;
}

as5600_err_t as5600_get_angle_data_raw_bus(as5600_t const* as5600, uint16_t* raw)
{
    assert(as5600 && raw);

    as5600_angle_reg_t reg = {};

    as5600_err_t err = as5600_get_angle_reg(as5600, &reg);

    *raw = reg.angle;

    return err;
}

as5600_err_t as5600_get_zmco_reg(as5600_t const* as5600, as5600_zmco_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_ZMCO, &data, sizeof(data));

    reg->zmco = data & 0x03U;

    return err;
}

as5600_err_t as5600_get_zpos_reg(as5600_t const* as5600, as5600_zpos_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_ZPOS_H, data, sizeof(data));

    reg->zpos = (uint16_t)(((data[0] & 0x0FU) << 8U) | (data[1] & 0xFFU));

    return err;
}

as5600_err_t as5600_set_zpos_reg(as5600_t const* as5600, as5600_zpos_reg_t const* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_ZPOS_H, data, sizeof(data));

    data[0] &= ~0x0FU;
    data[1] &= ~0xFFU;

    data[0] |= (reg->zpos >> 8U) & 0x0FU;
    data[1] = reg->zpos & 0xFFU;

    err |= as5600_bus_write_data(as5600, AS5600_REG_ADDRESS_ZPOS_H, data, sizeof(data));

    return err;
}

as5600_err_t as5600_get_mpos_reg(as5600_t const* as5600, as5600_mpos_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_MPOS_H, data, sizeof(data));

    reg->mpos = (uint16_t)(((data[0] & 0x0FU) << 8U) | (data[1] & 0xFFU));

    return err;
}

as5600_err_t as5600_set_mpos_reg(as5600_t const* as5600, as5600_mpos_reg_t const* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_MPOS_H, data, sizeof(data));

    data[0] &= ~0x0FU;
    data[1] &= ~0xFFU;

    data[0] |= (reg->mpos >> 8U) & 0x0FU;
    data[1] = reg->mpos & 0xFFU;

    err |= as5600_bus_write_data(as5600, AS5600_REG_ADDRESS_MPOS_H, data, sizeof(data));

    return err;
}

as5600_err_t as5600_get_mang_reg(as5600_t const* as5600, as5600_mang_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_MANG_H, data, sizeof(data));

    reg->mang = (uint16_t)(((data[0] & 0x0FU) << 8U) | (data[1] & 0xFFU));

    return err;
}

as5600_err_t as5600_set_mang_reg(as5600_t const* as5600, as5600_mang_reg_t const* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_MANG_H, data, sizeof(data));

    data[0] &= ~0x0FU;
    data[1] &= ~0xFFU;

    data[0] |= (reg->mang >> 8U) & 0x0FU;
    data[1] = reg->mang & 0xFFU;

    err |= as5600_bus_write_data(as5600, AS5600_REG_ADDRESS_MANG_H, data, sizeof(data));

    return err;
}

as5600_err_t as5600_get_conf_reg(as5600_t const* as5600, as5600_conf_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_CONF_H, data, sizeof(data));

    reg->wd = (data[0] >> 5U) & 0x01U;
    reg->fth = (data[0] >> 2U) & 0x07U;
    reg->sf = data[0] & 0x03U;
    reg->pwmf = (data[1] >> 6U) & 0x03U;
    reg->outs = (data[1] >> 4U) & 0x03U;
    reg->hyst = (data[1] >> 2U) & 0x03U;
    reg->pm = data[1] & 0x03U;

    return err;
}

as5600_err_t as5600_set_conf_reg(as5600_t const* as5600, as5600_conf_reg_t const* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_CONF_H, data, sizeof(data));

    data[0] &= ~((0x01U << 5U) | (0x07U << 2U) | 0x03U);
    data[1] &= ~((0x03U << 6U) | (0x03U << 4U) | (0x03U << 2U) | 0x03U);

    data[0] |= (reg->wd & 0x01U) << 5U;
    data[0] |= (reg->fth & 0x07U) << 2U;
    data[0] |= reg->sf & 0x03U;
    data[1] |= (reg->pwmf & 0x03U) << 6U;
    data[1] |= (reg->outs & 0x03U) << 4U;
    data[1] |= (reg->hyst & 0x03U) << 2U;
    data[1] |= reg->pm & 0x03U;

    err |= as5600_bus_write_data(as5600, AS5600_REG_ADDRESS_CONF_H, data, sizeof(data));

    return err;
}

as5600_err_t as5600_get_raw_angle_reg(as5600_t const* as5600, as5600_raw_angle_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err =
        as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_RAW_ANGLE_H, data, sizeof(data));

    reg->raw_angle = (uint16_t)(((data[0] & 0x0FU) << 8U) | (data[1] & 0xFFU));

    return err;
}

as5600_err_t as5600_get_angle_reg(as5600_t const* as5600, as5600_angle_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_ANGLE_H, data, sizeof(data));

    reg->angle = (uint16_t)(((data[0] & 0x0FU) << 8U) | (data[1] & 0xFFU));

    return err;
}

as5600_err_t as5600_get_status_reg(as5600_t const* as5600, as5600_status_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_STATUS, &data, sizeof(data));

    reg->md = (data >> 5U) & 0x01U;
    reg->ml = (data >> 4U) & 0x01U;
    reg->mh = (data >> 3U) & 0x01U;

    return err;
}

as5600_err_t as5600_get_agc_reg(as5600_t const* as5600, as5600_agc_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data = {};

    as5600_err_t err = as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_AGC, &data, sizeof(data));

    reg->agc = data & 0xFFU;

    return err;
}

as5600_err_t as5600_get_magnitude_reg(as5600_t const* as5600, as5600_magnitude_reg_t* reg)
{
    assert(as5600 && reg);

    uint8_t data[2] = {};

    as5600_err_t err =
        as5600_bus_read_data(as5600, AS5600_REG_ADDRESS_MAGNITUDE_H, data, sizeof(data));

    reg->magnitude = (uint16_t)(((data[0] & 0x0FU) << 8U) | (data[1] & 0xFFU));

    return err;
}

as5600_err_t as5600_send_burn_angle_cmd(as5600_t const* as5600)
{
    assert(as5600);

    uint8_t data = AS5600_CMD_BURN_ANGLE;

    return as5600_bus_write_data(as5600, data, NULL, 0UL);
}

as5600_err_t as5600_send_burn_setting_cmd(as5600_t const* as5600)
{
    assert(as5600);

    uint8_t data = AS5600_CMD_BURN_SETTING;

    return as5600_bus_write_data(as5600, data, NULL, 0UL);
}
