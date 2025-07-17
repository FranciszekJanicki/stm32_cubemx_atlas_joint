#include "drv8825.h"
#include "drv8825_config.h"
#include <assert.h>
#include <stdbool.h>
#include <string.h>

static drv8825_err_t drv8825_pwm_initialize(drv8825_t const* drv8825)
{
    return drv8825->interface.pwm_initialize
               ? drv8825->interface.pwm_initialize(drv8825->interface.pwm_user)
               : DRV8825_ERR_NULL;
}

static drv8825_err_t drv8825_pwm_deinitialize(drv8825_t const* drv8825)
{
    return drv8825->interface.pwm_deinitialize
               ? drv8825->interface.pwm_deinitialize(drv8825->interface.pwm_user)
               : DRV8825_ERR_NULL;
}

static drv8825_err_t drv8825_pwm_start(drv8825_t const* drv8825)
{
    return drv8825->interface.pwm_start ? drv8825->interface.pwm_start(drv8825->interface.pwm_user)
                                        : DRV8825_ERR_NULL;
}

static drv8825_err_t drv8825_pwm_stop(drv8825_t const* drv8825)
{
    return drv8825->interface.pwm_stop ? drv8825->interface.pwm_stop(drv8825->interface.pwm_user)
                                       : DRV8825_ERR_NULL;
}

static drv8825_err_t drv8825_pwm_set_frequency(drv8825_t const* drv8825, uint32_t frequency)
{
    return drv8825->interface.pwm_set_frequency
               ? drv8825->interface.pwm_set_frequency(drv8825->interface.pwm_user, frequency)
               : DRV8825_ERR_NULL;
}

static drv8825_err_t drv8825_gpio_initialize(drv8825_t const* drv8825)
{
    return drv8825->interface.gpio_initialize
               ? drv8825->interface.gpio_initialize(drv8825->interface.gpio_user)
               : DRV8825_ERR_NULL;
}

static drv8825_err_t drv8825_gpio_deinitialize(drv8825_t const* drv8825)
{
    return drv8825->interface.gpio_deinitialize
               ? drv8825->interface.gpio_deinitialize(drv8825->interface.gpio_user)
               : DRV8825_ERR_NULL;
}

static drv8825_err_t drv8825_gpio_write_pin(drv8825_t const* drv8825, uint32_t pin, bool state)
{
    return drv8825->interface.gpio_write_pin
               ? drv8825->interface.gpio_write_pin(drv8825->interface.gpio_user, pin, state)
               : DRV8825_ERR_NULL;
}

drv8825_err_t drv8825_initialize(drv8825_t const* drv8825,
                                 drv8825_config_t const* config,
                                 drv8825_interface_t const* interface)
{
    assert(drv8825 && config && interface);

    memset(drv8825, 0, sizeof(*drv8825));
    memcpy(&drv8825->config, config, sizeof(*config));
    memcpy(&drv8825->interface, interface, sizeof(*interface));

    drv8825_err_t err = drv8825_pwm_initialize(drv8825);
    err |= drv8825_gpio_initialize(drv8825);

    return err;
}

drv8825_err_t drv8825_deinitialize(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_deinitialize(drv8825);
    err |= drv8825_pwm_deinitialize(drv8825);

    return err;
}

drv8825_err_t drv8825_set_frequency(drv8825_t const* drv8825, uint32_t frequency)
{
    assert(drv8825);

    return drv8825_pwm_set_frequency(drv8825, frequency);
}

drv8825_err_t drv8825_set_microstep(drv8825_t const* drv8825, drv8825_microstep_t microstep)
{
    assert(drv8825);

    switch (microstep) {
        case DRV8825_MICROSTEP_FULL:
            return drv8825_set_full_microstep(drv8825);
        case DRV8825_MICROSTEP_HALF:
            return drv8825_set_half_microstep(drv8825);
        case DRV8825_MICROSTEP_QUARTER:
            return drv8825_set_quarter_microstep(drv8825);
        case DRV8825_MICROSTEP_EIGHTH:
            return drv8825_set_eighth_microstep(drv8825);
        case DRV8825_MICROSTEP_SIXTEENTH:
            return drv8825_set_sixteenth_microstep(drv8825);
        default:
            return DRV8825_ERR_FAIL;
    }
}

drv8825_err_t drv8825_set_full_microstep(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode0, false);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode1, false);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode2, false);

    return err;
}

drv8825_err_t drv8825_set_half_microstep(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode0, true);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode1, false);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode2, false);

    return err;
}

drv8825_err_t drv8825_set_quarter_microstep(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode0, false);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode1, true);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode2, false);

    return err;
}

drv8825_err_t drv8825_set_eighth_microstep(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode0, true);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode1, true);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode2, false);

    return err;
}

drv8825_err_t drv8825_set_sixteenth_microstep(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode0, false);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode1, false);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode2, true);

    return err;
}

drv8825_err_t drv8825_set_thirtysecondth_microstep(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode0, true);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode1, false);
    err |= drv8825_gpio_write_pin(drv8825, drv8825->config.pin_mode2, true);

    return err;
}

drv8825_err_t drv8825_set_direction(drv8825_t const* drv8825, drv8825_direction_t direction)
{
    assert(drv8825);

    switch (direction) {
        case DRV8825_DIRECTION_FORWARD:
            return drv8825_set_forward_direction(drv8825);
        case DRV8825_DIRECTION_BACKWARD:
            return drv8825_set_backward_direction(drv8825);
        case DRV8825_DIRECTION_STOP:
            return drv8825_set_stop_direction(drv8825);
        default:
            return DRV8825_ERR_FAIL;
    }
}

drv8825_err_t drv8825_set_forward_direction(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_dir, false);
    err |= drv8825_pwm_start(drv8825);

    return err;
}

drv8825_err_t drv8825_set_backward_direction(drv8825_t const* drv8825)
{
    assert(drv8825);

    drv8825_err_t err = drv8825_gpio_write_pin(drv8825, drv8825->config.pin_dir, true);
    err |= drv8825_pwm_start(drv8825);

    return err;
}

drv8825_err_t drv8825_set_stop_direction(drv8825_t const* drv8825)
{
    assert(drv8825);

    return drv8825_pwm_stop(drv8825);
}

drv8825_err_t drv8825_set_decay(drv8825_t const* drv8825, drv8825_decay_t decay)
{
    assert(drv8825);

    switch (decay) {
        case DRV8825_DECAY_FAST:
            return drv8825_set_fast_decay(drv8825);
        case DRV8825_DECAY_SLOW:
            return drv8825_set_slow_decay(drv8825);
        default:
            return DRV8825_ERR_FAIL;
    }
}

drv8825_err_t drv8825_set_fast_decay(drv8825_t const* drv8825)
{
    assert(drv8825);

    return drv8825_gpio_write_pin(drv8825, drv8825->config.pin_decay, true);
}

drv8825_err_t drv8825_set_slow_decay(drv8825_t const* drv8825)
{
    assert(drv8825);

    return drv8825_gpio_write_pin(drv8825, drv8825->config.pin_decay, false);
}

drv8825_err_t drv8825_set_reset(drv8825_t const* drv8825, bool reset)
{
    assert(drv8825);

    return drv8825_gpio_write_pin(drv8825, drv8825->config.pin_enable, !reset);
}

drv8825_err_t drv8825_set_enable(drv8825_t const* drv8825, bool enable)
{
    assert(drv8825);

    return drv8825_gpio_write_pin(drv8825, drv8825->config.pin_enable, !enable);
}

drv8825_err_t drv8825_set_sleep(drv8825_t const* drv8825, bool sleep)
{
    assert(drv8825);

    return drv8825_gpio_write_pin(drv8825, drv8825->config.pin_sleep, !sleep);
}
