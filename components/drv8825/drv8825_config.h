#ifndef DRV8825_DRV8825_CONFIG_H
#define DRV8825_DRV8825_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DRV8825_ERR_OK = 0,
    DRV8825_ERR_FAIL = 1 << 0,
    DRV8825_ERR_NULL = 1 << 1,
} drv8825_err_t;

typedef enum {
    DRV8825_MICROSTEP_FULL,
    DRV8825_MICROSTEP_HALF,
    DRV8825_MICROSTEP_QUARTER,
    DRV8825_MICROSTEP_EIGHTH,
    DRV8825_MICROSTEP_SIXTEENTH,
    DRV8825_MICROSTEP_THIRTYSECONDTH,
} drv8825_microstep_t;

typedef enum {
    DRV8825_DECAY_FAST,
    DRV8825_DECAY_SLOW,
} drv8825_decay_t;

typedef enum {
    DRV8825_DIRECTION_FORWARD,
    DRV8825_DIRECTION_BACKWARD,
    DRV8825_DIRECTION_STOP,
} drv8825_direction_t;

inline float drv8825_microstep_to_fraction(drv8825_microstep_t microstep)
{
    switch (microstep) {
        case DRV8825_MICROSTEP_FULL:
            return 1.0F;
        case DRV8825_MICROSTEP_HALF:
            return 0.5F;
        case DRV8825_MICROSTEP_QUARTER:
            return 0.25F;
        case DRV8825_MICROSTEP_EIGHTH:
            return 0.125F;
        case DRV8825_MICROSTEP_SIXTEENTH:
            return 0.0625F;
        case DRV8825_MICROSTEP_THIRTYSECONDTH:
            return 0.03125F;
        default:
            return 0.0F;
    }
}

typedef struct {
    uint32_t pin_mode0;
    uint32_t pin_mode1;
    uint32_t pin_mode2;
    uint32_t pin_reset;
    uint32_t pin_sleep;
    uint32_t pin_dir;
    uint32_t pin_enable;
    uint32_t pin_decay;
} drv8825_config_t;

typedef struct {
    void* gpio_user;
    drv8825_err_t (*gpio_initialize)(void*);
    drv8825_err_t (*gpio_deinitialize)(void*);
    drv8825_err_t (*gpio_write_pin)(void*, uint32_t, bool);

    void* pwm_user;
    drv8825_err_t (*pwm_initialize)(void*);
    drv8825_err_t (*pwm_deinitialize)(void*);
    drv8825_err_t (*pwm_start)(void*);
    drv8825_err_t (*pwm_stop)(void*);
    drv8825_err_t (*pwm_set_frequency)(void*, uint32_t);
} drv8825_interface_t;

#endif // DRV8825_DRV8825_CONFIG_H