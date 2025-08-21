#ifndef JOINT_TASK_JOINT_MANAGER_H
#define JOINT_TASK_JOINT_MANAGER_H

#include "FreeRTOS.h"
#include "a4988.h"
#include "as5600.h"
#include "common.h"
#include "ina226.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "queue.h"
#include "semphr.h"
#include "step_motor.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    GPIO_TypeDef* a4988_gpio;
    uint32_t a4988_dir_pin;
    TIM_HandleTypeDef* a4988_pwm_timer;
    uint32_t a4988_pwm_channel;

    I2C_HandleTypeDef* ina226_i2c_bus;
    uint16_t ina226_i2c_address;

    GPIO_TypeDef* as5600_gpio;
    uint32_t as5600_dir_pin;
    I2C_HandleTypeDef* as5600_i2c_bus;
    uint16_t as5600_i2c_address;
} joint_config_t;

typedef struct {
    bool is_running;
    bool has_fault;

    atlas_joint_measure_t measure;
    atlas_joint_reference_t reference;

    as5600_t as5600;
    a4988_t a4988;
    ina226_t ina226;
    step_motor_t motor;
    pid_regulator_t regulator;
    motor_driver_t driver;

    joint_config_t config;
} joint_manager_t;

typedef struct {
    float32_t prop_gain;
    float32_t int_gain;
    float32_t dot_gain;
    float32_t sat_gain;
    float32_t dead_error;
    float32_t min_speed;
    float32_t max_speed;
    float32_t min_position;
    float32_t max_position;
    float32_t step_change;
    float32_t current_limit;
} joint_parameters_t;

atlas_err_t joint_manager_initialize(joint_manager_t* manager,
                                     joint_config_t const* config,
                                     joint_parameters_t const* parameters);
atlas_err_t joint_manager_process(joint_manager_t* manager);

#endif // JOINT_TASK_JOINT_MANAGER_H
