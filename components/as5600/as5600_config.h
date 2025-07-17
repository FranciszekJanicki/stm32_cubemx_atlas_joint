#ifndef AS5600_AS5600_CONFIG_H
#define AS5600_AS5600_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { AS5600_SLAVE_ADDRESS = 0x36 };

typedef float float32_t;

typedef enum {
    AS5600_ERR_OK = 0,
    AS5600_ERR_FAIL = 1 << 0,
    AS5600_ERR_NULL = 1 << 1,
} as5600_err_t;

typedef enum {
    AS5600_REG_ADDRESS_ZMCO = 0x00,
    AS5600_REG_ADDRESS_ZPOS_H = 0x01,
    AS5600_REG_ADDRESS_ZPOS_L = 0x02,
    AS5600_REG_ADDRESS_MPOS_H = 0x03,
    AS5600_REG_ADDRESS_MPOS_L = 0x04,
    AS5600_REG_ADDRESS_MANG_H = 0x05,
    AS5600_REG_ADDRESS_MANG_L = 0x06,
    AS5600_REG_ADDRESS_CONF_H = 0x07,
    AS5600_REG_ADDRESS_CONF_L = 0x08,
    AS5600_REG_ADDRESS_RAW_ANGLE_H = 0x0C,
    AS5600_REG_ADDRESS_RAW_ANGLE_L = 0x0D,
    AS5600_REG_ADDRESS_ANGLE_H = 0x0E,
    AS5600_REG_ADDRESS_ANGLE_L = 0x0F,
    AS5600_REG_ADDRESS_STATUS = 0x0B,
    AS5600_REG_ADDRESS_AGC = 0x1A,
    AS5600_REG_ADDRESS_MAGNITUDE_H = 0x1B,
    AS5600_REG_ADDRESS_MAGNITUDE_L = 0x1C,
} as5600_reg_address_t;

typedef enum {
    AS5600_CMD_BURN_ANGLE = 0x80,
    AS5600_CMD_BURN_SETTING = 0x40,
} as5600_cmd_t;

typedef enum {
    AS5600_DIRECTION_CW = 0,
    AS5600_DIRECTION_CCW = 1,
} as5600_direction_t;

typedef enum {
    AS5600_PROGRAM_OPTION_A,
    AS5600_PROGRAM_OPTION_B,
} as5600_program_option_t;

typedef enum {
    AS5600_PWM_FREQUENCY_115HZ = 0b00,
    AS5600_PWM_FREQUENCY_230HZ = 0b01,
    AS5600_PWM_FREQUENCY_460HZ = 0b10,
    AS5600_PWM_FREQUENCY_920HZ = 0b11,
} as5600_pwm_frequency_t;

typedef enum {
    AS5600_SETTLING_TIME_2MS2 = 0b00,
    AS5600_SETTLING_TIME_1MS1 = 0b01,
    AS5600_SETTLING_TIME_0MS55 = 0b10,
    AS5600_SETTLING_TIME_0MS286 = 0b11,
} as5600_settling_time_t;

typedef enum {
    AS5600_POWER_MODE_NOM = 0b00,
    AS5600_POWER_MODE_LPM1 = 0b01,
    AS5600_POWER_MODE_LPM2 = 0b10,
    AS5600_POWER_MODE_LPM3 = 0b11,
} as5600_power_mode_t;

typedef enum {
    AS5600_HYSTERESIS_OFF = 0b00,
    AS5600_HYSTERESIS_1LSB = 0b01,
    AS5600_HYSTERESIS_2LSBS = 0b10,
    AS5600_HYSTERESIS_3LSBS = 0b11,
} as5600_hysteresis_t;

typedef enum {
    AS5600_OUTPUT_STAGE_ADC_FULL = 0b00,
    AS5600_OUTPUT_STAGE_ADC_REDUCED = 0b01,
    AS5600_OUTPUT_STAGE_PWM = 0b10,
} as5600_output_stage_t;

typedef enum {
    AS5600_SLOW_FILTER_X16 = 0b00,
    AS5600_SLOW_FILTER_X8 = 0b01,
    AS5600_SLOW_FILTER_X4 = 0b10,
    AS5600_SLOW_FILTER_X2 = 0b11,
} as5600_slow_filter_t;

typedef enum {
    AS5600_FAST_FILTER_THRESH_SLOW = 0b000,
    AS5600_FAST_FILTER_THRESH_6LSBS = 0b001,
    AS5600_FAST_FILTER_THRESH_7LSBS = 0b010,
    AS5600_FAST_FILTER_THRESH_9LSBS = 0b011,
    AS5600_FAST_FILTER_THRESH_18LSBS = 0b100,
    AS5600_FAST_FILTER_THRESH_21LSBS = 0b101,
    AS5600_FAST_FILTER_THRESH_24LSBS = 0b110,
    AS5600_FAST_FILTER_THRESH_10LSBS = 0b111,
} as5600_fast_filter_thresh_t;

typedef enum {
    AS5600_WATCHDOG_OFF = 0b0,
    AS5600_WATCHDOG_ON = 0b1,
} as5600_watchdog_t;

typedef struct {
    float32_t min_angle;
    float32_t max_angle;
    uint32_t dir_pin;
    uint32_t pgo_pin;
} as5600_config_t;

typedef struct {
    void* bus_user;
    as5600_err_t (*bus_initialize)(void*);
    as5600_err_t (*bus_deinitialize)(void*);
    as5600_err_t (*bus_write_data)(void*, uint8_t, uint8_t const*, size_t);
    as5600_err_t (*bus_read_data)(void*, uint8_t, uint8_t*, size_t);

    void* adc_user;
    as5600_err_t (*adc_initialize)(void*);
    as5600_err_t (*adc_deinitialize)(void*);
    as5600_err_t (*adc_get_conversion)(void*, uint16_t*);

    void* gpio_user;
    as5600_err_t (*gpio_initialize)(void*);
    as5600_err_t (*gpio_deinitialize)(void*);
    as5600_err_t (*gpio_write_pin)(void*, uint32_t, bool);
} as5600_interface_t;

#endif // AS5600_AS5600_CONFIG_H