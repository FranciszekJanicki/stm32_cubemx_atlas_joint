#include "motor_driver.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static motor_driver_err_t motor_driver_motor_initialize(motor_driver_t const* driver)
{
    return driver->interface.motor_initialize
               ? driver->interface.motor_initialize(driver->interface.motor_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_motor_deinitialize(motor_driver_t const* driver)
{
    return driver->interface.motor_deinitialize
               ? driver->interface.motor_deinitialize(driver->interface.motor_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_motor_set_speed(motor_driver_t const* driver,
                                                       float32_t speed)
{
    return driver->interface.motor_set_speed
               ? driver->interface.motor_set_speed(driver->interface.motor_user, speed)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_encoder_initialize(motor_driver_t const* driver)
{
    return driver->interface.encoder_initialize
               ? driver->interface.encoder_initialize(driver->interface.encoder_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_encoder_deinitialize(motor_driver_t const* driver)
{
    return driver->interface.encoder_deinitialize
               ? driver->interface.encoder_deinitialize(driver->interface.encoder_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_encoder_get_position(motor_driver_t const* driver,
                                                            float32_t* position)
{
    return driver->interface.encoder_get_position
               ? driver->interface.encoder_get_position(driver->interface.encoder_user, position)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_regulator_initialize(motor_driver_t const* driver)
{
    return driver->interface.regulator_initialize
               ? driver->interface.regulator_initialize(driver->interface.regulator_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_regulator_deinitialize(motor_driver_t const* driver)
{
    return driver->interface.regulator_deinitialize
               ? driver->interface.regulator_deinitialize(driver->interface.regulator_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_regulator_get_control(motor_driver_t const* driver,
                                                             float32_t error,
                                                             float32_t* control,
                                                             float32_t delta_time)
{
    return driver->interface.regulator_get_control
               ? driver->interface.regulator_get_control(driver->interface.regulator_user,
                                                         error,
                                                         control,
                                                         delta_time)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_fault_initialize(motor_driver_t const* driver)
{
    return driver->interface.fault_initialize
               ? driver->interface.fault_initialize(driver->interface.fault_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_fault_deinitialize(motor_driver_t const* driver)
{
    return driver->interface.fault_deinitialize
               ? driver->interface.fault_deinitialize(driver->interface.fault_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_fault_get_current(motor_driver_t const* driver,
                                                         float32_t* current)
{
    return driver->interface.fault_get_current
               ? driver->interface.fault_get_current(driver->interface.fault_user, current)
               : MOTOR_DRIVER_ERR_NULL;
}

static inline float32_t motor_driver_clamp_position(motor_driver_t const* driver,
                                                    float32_t position)
{
    if (position < driver->config.min_position) {
        position = driver->config.min_position;
    } else if (position > driver->config.max_position) {
        position = driver->config.max_position;
    }

    return position;
}

static inline float32_t motor_driver_clamp_speed(motor_driver_t const* driver, float32_t speed)
{
    if (speed != 0.0F) {
        if (fabsf(speed) < driver->config.min_speed) {
            speed = copysignf(driver->config.min_speed, speed);
        } else if (fabsf(speed) > driver->config.max_speed) {
            speed = copysignf(driver->config.max_speed, speed);
        }
    }

    return speed;
}

static inline bool motor_driver_has_fault(motor_driver_t const* driver, float32_t current)
{
    return current > driver->config.max_current;
}

motor_driver_err_t motor_driver_initialize(motor_driver_t* driver,
                                           motor_driver_config_t const* config,
                                           motor_driver_interface_t const* interface)
{
    assert(driver && config && interface);

    memset(driver, 0, sizeof(*driver));
    memcpy(&driver->config, config, sizeof(*config));
    memcpy(&driver->interface, interface, sizeof(*interface));

    motor_driver_err_t err = motor_driver_motor_initialize(driver);
    err |= motor_driver_encoder_initialize(driver);
    err |= motor_driver_regulator_initialize(driver);
    err |= motor_driver_fault_initialize(driver);

    return err;
}

motor_driver_err_t motor_driver_deinitialize(motor_driver_t* driver)
{
    assert(driver);

    motor_driver_err_t err = motor_driver_motor_deinitialize(driver);
    err |= motor_driver_encoder_deinitialize(driver);
    err |= motor_driver_regulator_deinitialize(driver);
    err |= motor_driver_fault_deinitialize(driver);

    memset(driver, 0, sizeof(*driver));

    return err;
}

motor_driver_err_t motor_driver_set_position(motor_driver_t* driver,
                                             float32_t position,
                                             float32_t delta_time,
                                             float32_t* measurement)
{
    assert(driver);

    float32_t current;
    motor_driver_err_t err = motor_driver_fault_get_current(driver, &current);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    if (motor_driver_has_fault(driver, current)) {
        return MOTOR_DRIVER_ERR_FAULT;
    }

    float32_t measured_position;
    err = motor_driver_encoder_get_position(driver, &measured_position);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    position = motor_driver_clamp_position(driver, position);
    float32_t error_position = position - measured_position;

    float32_t control_speed;
    err = motor_driver_regulator_get_control(driver, error_position, &control_speed, delta_time);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    control_speed = motor_driver_clamp_speed(driver, control_speed);
    err = motor_driver_motor_set_speed(driver, control_speed);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    printf("current: %f, position: %f, measured_position: %f, error_position: "
           "%f, control_speed: "
           "%f\n\r ",
           (double)current,
           (double)position,
           (double)measured_position,
           (double)error_position,
           (double)control_speed);

    if (measurement != NULL) {
        *measurement = measured_position;
    }

    return err;
}
