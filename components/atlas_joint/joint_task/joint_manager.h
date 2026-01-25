#ifndef JOINT_TASK_JOINT_MANAGER_H
#define JOINT_TASK_JOINT_MANAGER_H

#include "FreeRTOS.h"
#include "as5600.h"
#include "common.h"
#include "drv8825.h"
#include "ina226.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "queue.h"
#include "semphr.h"
#include "step_motor.h"
#include "stm32f411xe.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    GPIO_TypeDef* drv8825_en_gpio;
    uint16_t drv8825_en_pin;

    GPIO_TypeDef* drv8825_dir_gpio;
    uint16_t drv8825_dir_pin;

    GPIO_TypeDef* drv8825_m0_gpio;
    uint16_t drv8825_m0_pin;

    GPIO_TypeDef* drv8825_m1_gpio;
    uint16_t drv8825_m1_pin;

    GPIO_TypeDef* drv8825_m2_gpio;
    uint16_t drv8825_m2_pin;

    TIM_HandleTypeDef* drv8825_pwm_timer;
    uint16_t drv8825_pwm_channel;

    I2C_HandleTypeDef* ina226_i2c_bus;
    uint16_t ina226_i2c_address;

    GPIO_TypeDef* as5600_dir_gpio;
    uint16_t as5600_dir_pin;

    I2C_HandleTypeDef* as5600_i2c_bus;
    uint16_t as5600_i2c_address;

#ifdef JOINT_TEST
    TIM_HandleTypeDef* joint_test_delta_timer;
    GPIO_TypeDef* joint_test_home_confirm_gpio;
    uint16_t joint_test_home_confirm_pin;
#endif
} joint_config_t;

typedef atlas_joint_parameters_t joint_parameters_t;

typedef struct {
    bool is_running;

    atlas_joint_state_t state;
    atlas_joint_measure_t measure;
    atlas_joint_reference_t reference;
    atlas_joint_parameters_t parameters;

    as5600_t as5600;
    drv8825_t drv8825;
    ina226_t ina226;
    step_motor_t motor;
    pid_regulator_t regulator;
    motor_driver_t driver;

    joint_config_t config;
} joint_manager_t;

atlas_err_t joint_manager_initialize(joint_manager_t* manager,
                                     joint_config_t const* config,
                                     joint_parameters_t const* parameters);
atlas_err_t joint_manager_process(joint_manager_t* manager);

#endif // JOINT_TASK_JOINT_MANAGER_H
