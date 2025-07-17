#ifndef MOTOR_DRIVER_MOTOR_DRIVER_CONFIG_H
#define MOTOR_DRIVER_MOTOR_DRIVER_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef float float32_t;

typedef enum {
    MOTOR_DRIVER_ERR_OK = 0,
    MOTOR_DRIVER_ERR_FAIL = 1 << 0,
    MOTOR_DRIVER_ERR_NULL = 1 << 1,
    MOTOR_DRIVER_ERR_FAULT = 1 << 2,
} motor_driver_err_t;

typedef struct {
    float32_t position;
    float32_t speed;
    float32_t fault;
} motor_driver_state_t;

typedef struct {
    float32_t min_position;
    float32_t max_position;
    float32_t max_speed;
    float32_t min_speed;
    float32_t max_current;
} motor_driver_config_t;

typedef struct {
    void* motor_user;
    motor_driver_err_t (*motor_initialize)(void*);
    motor_driver_err_t (*motor_deinitialize)(void*);
    motor_driver_err_t (*motor_set_speed)(void*, float32_t);

    void* encoder_user;
    motor_driver_err_t (*encoder_initialize)(void*);
    motor_driver_err_t (*encoder_deinitialize)(void*);
    motor_driver_err_t (*encoder_get_position)(void*, float32_t*);

    void* regulator_user;
    motor_driver_err_t (*regulator_initialize)(void*);
    motor_driver_err_t (*regulator_deinitialize)(void*);
    motor_driver_err_t (*regulator_get_control)(void*, float32_t, float32_t*, float32_t);

    void* fault_user;
    motor_driver_err_t (*fault_initialize)(void*);
    motor_driver_err_t (*fault_deinitialize)(void*);
    motor_driver_err_t (*fault_get_current)(void*, float32_t*);
} motor_driver_interface_t;

#endif // MOTOR_DRIVER_MOTOR_DRIVER_CONFIG_H
