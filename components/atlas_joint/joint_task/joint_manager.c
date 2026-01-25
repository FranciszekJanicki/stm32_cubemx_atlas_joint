#include "joint_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "drv8825.h"
#include "event.h"
#include "manager.h"
#include "motor_driver.h"
#include "notify.h"
#include "pid_regulator.h"
#include "step_motor.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static char const* const TAG = "atlas_joint:joint_manager";

float drv8825_microstep_to_fraction(drv8825_microstep_t microstep);

static inline bool frequency_to_prescaler_and_period(uint32_t frequency_hz,
                                                     uint32_t clock_hz,
                                                     uint32_t max_prescaler,
                                                     uint32_t max_period,
                                                     uint32_t* prescaler,
                                                     uint32_t* period)
{
    if (frequency_hz == 0U || !prescaler || !period) {
        return false;
    }

    uint32_t temp_prescaler = 0U;
    uint32_t temp_period = clock_hz / frequency_hz;

    while (temp_period > max_period && temp_prescaler < max_prescaler) {
        temp_prescaler++;
        temp_period = clock_hz / ((temp_prescaler + 1U) * frequency_hz);
    }
    if (temp_period > max_period) {
        temp_period = max_period;
        temp_prescaler = (clock_hz / (temp_period * frequency_hz)) - 1U;
    }
    if (temp_prescaler > max_prescaler) {
        temp_prescaler = max_prescaler;
    }

    *prescaler = temp_prescaler;
    *period = temp_period;

    return true;
}

static inline drv8825_err_t drv8825_gpio_initialize(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    return DRV8825_ERR_OK;
}

static inline drv8825_err_t drv8825_gpio_deinitialize(void* user)
{
    return DRV8825_ERR_OK;
}

static inline drv8825_err_t drv8825_gpio_write_pin(void* user,
                                                   uint32_t pin,
                                                   bool state)
{
    joint_config_t* config = (joint_config_t*)user;

    GPIO_TypeDef* port = nullptr;
    if (pin == config->drv8825_dir_pin)
        port = config->drv8825_dir_gpio;
    else if (pin == config->drv8825_en_pin)
        port = config->drv8825_en_gpio;
    else if (pin == config->drv8825_m2_pin)
        port = config->drv8825_m2_gpio;
    else if (pin == config->drv8825_m1_pin)
        port = config->drv8825_m1_gpio;
    else if (pin == config->drv8825_m0_pin)
        port = config->drv8825_m0_gpio;

    HAL_GPIO_WritePin(port, (uint16_t)pin, (GPIO_PinState)state);

    return DRV8825_ERR_OK;
}

static inline drv8825_err_t drv8825_pwm_initialize(void* user)
{
    return DRV8825_ERR_OK;
}

static inline drv8825_err_t drv8825_pwm_deinitialize(void* user)
{
    return DRV8825_ERR_OK;
}

static inline drv8825_err_t drv8825_pwm_start(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    return HAL_TIM_PWM_Start_IT(config->drv8825_pwm_timer,
                                config->drv8825_pwm_channel) == HAL_OK
               ? DRV8825_ERR_OK
               : DRV8825_ERR_FAIL;
}

static inline drv8825_err_t drv8825_pwm_stop(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    return HAL_TIM_PWM_Stop_IT(config->drv8825_pwm_timer,
                               config->drv8825_pwm_channel) == HAL_OK
               ? DRV8825_ERR_OK
               : DRV8825_ERR_FAIL;
}

static inline drv8825_err_t drv8825_pwm_set_frequency(void* user,
                                                      uint32_t frequency)
{
    joint_config_t* config = (joint_config_t*)user;

    uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        clock_hz *= 2;
    }

    uint32_t period;
    uint32_t prescaler;
    bool result = frequency_to_prescaler_and_period(frequency,
                                                    clock_hz,
                                                    0xFFFFU,
                                                    0xFFFFU,
                                                    &prescaler,
                                                    &period);

    if (!result || period >= 0xFFFFU || prescaler >= 0xFFFFU) {
        return DRV8825_ERR_FAIL;
    }

    uint32_t compare = (uint32_t)((float32_t)period / 2.0F);

    __HAL_TIM_DISABLE(config->drv8825_pwm_timer);
    __HAL_TIM_SET_COUNTER(config->drv8825_pwm_timer, 0U);
    __HAL_TIM_SET_PRESCALER(config->drv8825_pwm_timer, prescaler);
    __HAL_TIM_SET_AUTORELOAD(config->drv8825_pwm_timer, period);
    __HAL_TIM_SET_COMPARE(config->drv8825_pwm_timer,
                          config->drv8825_pwm_channel,
                          compare);
    __HAL_TIM_ENABLE(config->drv8825_pwm_timer);

    ATLAS_LOG(
        TAG,
        "clock: %u, frequency: %u, period: %u, prescaler: %u, compare: %u",
        clock_hz,
        frequency,
        period,
        prescaler,
        compare);

    return DRV8825_ERR_OK;
}

static inline as5600_err_t as5600_gpio_initialize(void* user)
{
    return AS5600_ERR_OK;
}

static inline as5600_err_t as5600_gpio_deinitialize(void* user)
{
    return AS5600_ERR_OK;
}

static inline as5600_err_t as5600_gpio_write_pin(void* user,
                                                 uint32_t pin,
                                                 bool state)
{
    joint_config_t* config = (joint_config_t*)user;

    GPIO_TypeDef* gpio = NULL;
    if (pin == config->as5600_dir_pin) {
        gpio = config->as5600_dir_gpio;
    }

    HAL_GPIO_WritePin(gpio, (uint16_t)pin, (GPIO_PinState)state);

    return AS5600_ERR_OK;
}

static inline as5600_err_t as5600_adc_initialize(void* user)
{
    return AS5600_ERR_OK;
}

static inline as5600_err_t as5600_adc_deinitialize(void* user)
{
    return AS5600_ERR_OK;
}

static inline as5600_err_t as5600_bus_initialize(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    return HAL_I2C_IsDeviceReady(config->as5600_i2c_bus,
                                 config->as5600_i2c_address << 1U,
                                 10U,
                                 1000U) == HAL_OK
               ? AS5600_ERR_OK
               : AS5600_ERR_FAIL;
}

static inline as5600_err_t as5600_bus_deinitialize(void* user)
{
    return AS5600_ERR_OK;
}

static inline as5600_err_t as5600_bus_write_data(void* user,
                                                 uint8_t address,
                                                 uint8_t const* data,
                                                 size_t data_size)
{
    joint_config_t* config = (joint_config_t*)user;

    return HAL_I2C_Mem_Write(config->as5600_i2c_bus,
                             config->as5600_i2c_address << 1U,
                             address,
                             I2C_MEMADD_SIZE_8BIT,
                             data,
                             data_size,
                             100) == HAL_OK
               ? AS5600_ERR_OK
               : AS5600_ERR_FAIL;
}

static inline as5600_err_t as5600_bus_read_data(void* user,
                                                uint8_t address,
                                                uint8_t* data,
                                                size_t data_size)
{
    joint_config_t* config = (joint_config_t*)user;

    return HAL_I2C_Mem_Read(config->as5600_i2c_bus,
                            config->as5600_i2c_address << 1U,
                            address,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            data_size,
                            100) == HAL_OK
               ? AS5600_ERR_OK
               : AS5600_ERR_FAIL;
}

static inline as5600_err_t as5600_initialize_chip(as5600_t* as5600,
                                                  float32_t min_angle,
                                                  float32_t max_angle,
                                                  bool magnet_polarity)
{
    as5600_status_reg_t status;
    as5600_err_t err = as5600_get_status_reg(as5600, &status);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    float32_t angle_range = (max_angle - min_angle);

    uint16_t min_raw = (uint16_t)(min_angle / angle_range * 4095.0F);
    uint16_t max_raw = (uint16_t)(max_angle / angle_range * 4095.0F);

    as5600_zpos_reg_t zpos = {.zpos = min_raw & 0x0FFF};
    err = as5600_set_zpos_reg(as5600, &zpos);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    as5600_mpos_reg_t mpos = {.mpos = max_raw & 0x0FFF};
    err = as5600_set_mpos_reg(as5600, &mpos);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    as5600_conf_reg_t conf = {.wd = AS5600_WATCHDOG_OFF,
                              .fth = AS5600_SLOW_FILTER_X16,
                              .sf = AS5600_SLOW_FILTER_X16,
                              .pwmf = AS5600_PWM_FREQUENCY_115HZ,
                              .outs = AS5600_FAST_FILTER_THRESH_SLOW,
                              .hyst = AS5600_HYSTERESIS_OFF,
                              .pm = AS5600_POWER_MODE_NOM};
    err = as5600_set_conf_reg(as5600, &conf);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    as5600_zmco_reg_t zmco;
    err = as5600_get_zmco_reg(as5600, &zmco);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    return as5600_set_direction(as5600, (as5600_direction_t)magnet_polarity);
}

static inline ina226_err_t ina226_bus_initialize(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    return HAL_I2C_IsDeviceReady(config->ina226_i2c_bus,
                                 config->ina226_i2c_address << 1U,
                                 3U,
                                 10U) == HAL_OK
               ? INA226_ERR_OK
               : INA226_ERR_FAIL;
}

static inline ina226_err_t ina226_bus_deinitialize(void* user)
{
    return INA226_ERR_OK;
}

static inline ina226_err_t ina226_bus_write_data(void* user,
                                                 uint8_t address,
                                                 uint8_t const* data,
                                                 size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_config_t* config = (joint_config_t*)user;

    return HAL_I2C_Mem_Write(config->ina226_i2c_bus,
                             config->ina226_i2c_address << 1U,
                             address,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t*)data,
                             data_size,
                             100) == HAL_OK
               ? INA226_ERR_OK
               : INA226_ERR_FAIL;
}

static inline ina226_err_t ina226_bus_read_data(void* user,
                                                uint8_t address,
                                                uint8_t* data,
                                                size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_config_t* config = (joint_config_t*)user;

    return HAL_I2C_Mem_Read(config->ina226_i2c_bus,
                            config->ina226_i2c_address << 1U,
                            address,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            data_size,
                            100) == HAL_OK
               ? INA226_ERR_OK
               : INA226_ERR_FAIL;
}

float32_t ina226_current_range_to_scale(float32_t);

static inline ina226_err_t ina226_initialize_chip(ina226_t* ina226,
                                                  float32_t min_current,
                                                  float32_t max_current)
{
    return INA226_ERR_OK;
}

static inline step_motor_err_t step_motor_device_initialize(void* user)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    drv8825_set_enable(&manager->drv8825, true);
    drv8825_set_direction(&manager->drv8825, DRV8825_DIRECTION_STOP);
    drv8825_set_microstep(&manager->drv8825, manager->parameters.microstep);

    return STEP_MOTOR_ERR_OK;
}

static inline step_motor_err_t step_motor_device_deinitialize(void* user)
{
    joint_manager_t* manager = (joint_manager_t*)user;

    drv8825_set_enable(&manager->drv8825, true);
    drv8825_set_direction(&manager->drv8825, DRV8825_DIRECTION_STOP);

    return STEP_MOTOR_ERR_OK;
}

static inline step_motor_err_t step_motor_device_set_frequency(
    void* user,
    uint32_t frequency)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    drv8825_set_frequency(&manager->drv8825, frequency);

    return STEP_MOTOR_ERR_OK;
}

static inline step_motor_err_t step_motor_device_set_direction(
    void* user,
    step_motor_direction_t direction)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    drv8825_set_direction(&manager->drv8825, (drv8825_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

static inline motor_driver_err_t motor_driver_motor_initialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_motor_deinitialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_motor_set_speed(void* user,
                                                              float32_t speed)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    step_motor_set_speed(&manager->motor, speed);

    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_encoder_initialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_encoder_deinitialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_encoder_get_position(
    void* user,
    float32_t* position)
{
    ATLAS_ASSERT(user && position);

    joint_manager_t* manager = (joint_manager_t*)user;

    as5600_get_angle_data_scaled_bus(&manager->as5600, position);

    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_regulator_initialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_regulator_deinitialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_regulator_get_control(
    void* user,
    float32_t error,
    float32_t* control,
    float32_t delta_time)
{
    ATLAS_ASSERT(user && control);

    joint_manager_t* manager = (joint_manager_t*)user;

    pid_regulator_get_sat_control(&manager->regulator,
                                  error,
                                  delta_time,
                                  control);

    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_fault_initialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_fault_deinitialize(void* user)
{
    return MOTOR_DRIVER_ERR_OK;
}

static inline motor_driver_err_t motor_driver_fault_get_current(
    void* user,
    float32_t* current)
{
    ATLAS_ASSERT(user && current);

    joint_manager_t* manager = (joint_manager_t*)user;

    *current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}

static inline bool joint_manager_has_joint_event(void)
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_JOINT)) > 0U;
}

#ifdef USE_LOG_TASK
static inline bool joint_manager_send_log_notify(log_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_LOG), notify, eSetBits) ==
           pdPASS;
}
#endif

#ifdef USE_WATCHDOG_TASK
static inline bool joint_manager_send_watchdog_notify(log_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_WATCHDOG),
                       notify,
                       eSetBits) == pdPASS;
}
#endif

static inline bool joint_manager_send_system_notify(system_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_SYSTEM),
                       (uint32_t)notify,
                       eSetBits) == pdPASS;
}

static inline bool joint_manager_send_system_event(system_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_SYSTEM),
                      event,
                      pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joint_manager_receive_joint_event(joint_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_JOINT),
                         event,
                         pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joint_manager_receive_joint_notify(joint_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0,
                           JOINT_NOTIFY_ALL,
                           (uint32_t*)notify,
                           pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joint_manager_initialize_chips(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    if (as5600_initialize_chip(&manager->as5600,
                               manager->parameters.min_position,
                               manager->parameters.max_position,
                               manager->parameters.magnet_polarity) !=
        AS5600_ERR_OK) {
        ATLAS_LOG(TAG, "Failed as5600_initialize_chip");

        //     return false;
    }

    // if (ina226_initialize_chip(&manager->ina226,
    //                            0.0F,
    //                            manager->parameters.current_limit) !=
    //     INA226_ERR_OK) {
    //     ATLAS_LOG(TAG, "Failed ina226_initialize_chip");

    //     return false;
    // }

    return true;
}

static inline bool joint_manager_deinitialize_chips(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
}

#ifdef JOINT_TEST
static inline bool joint_manager_start_joint_test_delta_timer(
    joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_TIM_Base_Start_IT(manager->config.joint_test_delta_timer) ==
           HAL_OK;
}

static inline bool joint_manager_stop_joint_test_delta_timer(
    joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_TIM_Base_Stop_IT(manager->config.joint_test_delta_timer) ==
           HAL_OK;
}
#endif

static inline bool joint_manager_initialize_drivers(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    if (as5600_initialize(
            &manager->as5600,
            &(as5600_config_t){.dir_pin = manager->config.as5600_dir_pin,
                               .max_angle = manager->parameters.max_position,
                               .min_angle = manager->parameters.min_position},
            &(as5600_interface_t){.gpio_user = &manager->config,
                                  .gpio_initialize = as5600_gpio_initialize,
                                  .gpio_deinitialize = as5600_gpio_deinitialize,
                                  .gpio_write_pin = as5600_gpio_write_pin,
                                  .adc_user = &manager->config,
                                  .adc_initialize = as5600_adc_initialize,
                                  .adc_deinitialize = as5600_adc_deinitialize,
                                  .bus_user = &manager->config,
                                  .bus_initialize = as5600_bus_initialize,
                                  .bus_deinitialize = as5600_bus_deinitialize,
                                  .bus_read_data = as5600_bus_read_data,
                                  .bus_write_data = as5600_bus_write_data}) !=
        AS5600_ERR_OK) {
        ATLAS_LOG(TAG, "Failed as5600_initialize");

        //   return false;
    }

    // if (ina226_initialize(
    //         &manager->ina226,
    //         &(ina226_config_t){.current_scale =
    //         ina226_current_range_to_scale(
    //                                manager->parameters.current_limit)},
    //         &(ina226_interface_t){.bus_user = &manager->config,
    //                               .bus_initialize =
    //                               ina226_bus_initialize,
    //                               .bus_deinitialize =
    //                               ina226_bus_deinitialize, .bus_read_data
    //                               = ina226_bus_read_data, .bus_write_data
    //                               = ina226_bus_write_data}) !=
    //     INA226_ERR_OK) {
    //     ATLAS_LOG(TAG, "Failed ina226_initialize");

    //     return false;
    // }

    if (drv8825_initialize(
            &manager->drv8825,
            &(drv8825_config_t){.pin_dir = manager->config.drv8825_dir_pin,
                                .pin_enable = manager->config.drv8825_en_pin,
                                .pin_mode0 = manager->config.drv8825_m0_pin,
                                .pin_mode1 = manager->config.drv8825_m1_pin,
                                .pin_mode2 = manager->config.drv8825_m2_pin},
            &(drv8825_interface_t){
                .gpio_user = &manager->config,
                .gpio_initialize = drv8825_gpio_initialize,
                .gpio_deinitialize = drv8825_gpio_deinitialize,
                .gpio_write_pin = drv8825_gpio_write_pin,
                .pwm_user = &manager->config,
                .pwm_initialize = drv8825_pwm_initialize,
                .pwm_deinitialize = drv8825_pwm_deinitialize,
                .pwm_start = drv8825_pwm_start,
                .pwm_stop = drv8825_pwm_stop,
                .pwm_set_frequency = drv8825_pwm_set_frequency}) !=
        DRV8825_ERR_OK) {
        ATLAS_LOG(TAG, "Failed drv8825_initialize");

        return false;
    }

    if (step_motor_initialize(
            &manager->motor,
            &(step_motor_config_t){
                .min_position = manager->parameters.min_position,
                .max_position = manager->parameters.max_position,
                .min_speed = manager->parameters.min_speed,
                .max_speed = manager->parameters.max_speed,
                .step_change = manager->parameters.step_change *
                               drv8825_microstep_to_fraction(
                                   manager->parameters.microstep)},
            &(step_motor_interface_t){
                .device_user = manager,
                .device_initialize = step_motor_device_initialize,
                .device_deinitialize = step_motor_device_deinitialize,
                .device_set_frequency = step_motor_device_set_frequency,
                .device_set_direction = step_motor_device_set_direction},
            0.0F) != STEP_MOTOR_ERR_OK) {
        ATLAS_LOG(TAG, "Failed step_motor_initialize");

        return false;
    }

    if (pid_regulator_initialize(
            &manager->regulator,
            &(pid_regulator_config_t){
                .prop_gain = manager->parameters.prop_gain,
                .int_gain = manager->parameters.int_gain,
                .dot_gain = manager->parameters.dot_gain,
                .sat_gain = manager->parameters.sat_gain,
                .min_control = manager->parameters.min_speed,
                .max_control = manager->parameters.max_speed,
                .dead_error = manager->parameters.dead_error}) !=
        PID_REGULATOR_ERR_OK) {
        ATLAS_LOG(TAG, "Failed pid_regulator_initialize");

        return false;
    }

    if (motor_driver_initialize(
            &manager->driver,
            &(motor_driver_config_t){
                .min_position = manager->parameters.min_position,
                .max_position = manager->parameters.max_position,
                .min_speed = manager->parameters.min_speed,
                .max_speed = manager->parameters.max_speed,
                .min_acceleration = manager->parameters.min_acceleration,
                .max_acceleration = manager->parameters.max_acceleration,
                .max_current = manager->parameters.current_limit},
            &(motor_driver_interface_t){
                .motor_user = manager,
                .motor_initialize = motor_driver_motor_initialize,
                .motor_deinitialize = motor_driver_motor_deinitialize,
                .motor_set_speed = motor_driver_motor_set_speed,
                .encoder_user = manager,
                .encoder_initialize = motor_driver_encoder_initialize,
                .encoder_deinitialize = motor_driver_encoder_deinitialize,
                .encoder_get_position = motor_driver_encoder_get_position,
                .regulator_user = manager,
                .regulator_initialize = motor_driver_regulator_initialize,
                .regulator_deinitialize = motor_driver_regulator_deinitialize,
                .regulator_get_control = motor_driver_regulator_get_control,
                .fault_user = manager,
                .fault_initialize = motor_driver_fault_initialize,
                .fault_deinitialize = motor_driver_fault_deinitialize,
                .fault_get_current = motor_driver_fault_get_current}) !=
        MOTOR_DRIVER_ERR_OK) {
        ATLAS_LOG(TAG, "Failed motor_driver_initialize");

        return false;
    }

    return true;
}

static inline bool joint_manager_deinitialize_drivers(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    if (as5600_deinitialize(&manager->as5600) != AS5600_ERR_OK) {
        ATLAS_LOG(TAG, "Failed as5600_deinitialize");

        return false;
    }

    // if (ina226_deinitialize(&manager->ina226) != INA226_ERR_OK) {
    //     ATLAS_LOG(TAG, "Failed ina226_deinitialize");

    //     return false;
    // }

    if (drv8825_deinitialize(&manager->drv8825) != DRV8825_ERR_OK) {
        ATLAS_LOG(TAG, "Failed drv8825_deinitialize");

        return false;
    }

    if (step_motor_deinitialize(&manager->motor) != STEP_MOTOR_ERR_OK) {
        ATLAS_LOG(TAG, "Failed step_motor_deinitialize");

        return false;
    }

    if (pid_regulator_deinitialize(&manager->regulator) !=
        PID_REGULATOR_ERR_OK) {
        ATLAS_LOG(TAG, "Failed pid_regulator_deinitialize");

        return false;
    }

    if (motor_driver_deinitialize(&manager->driver) != MOTOR_DRIVER_ERR_OK) {
        ATLAS_LOG(TAG, "Failed motor_driver_deinitialize");

        return false;
    }

    return true;
}

#ifdef JOINT_TEST

typedef struct {
    float q_start;
    float q_end;
    float t;
    float T;
    bool active;
} joint_ptp_t;

#define V_MAX 1.0f
#define A_MAX 0.5f
#define FACTOR_V 1.875f
#define FACTOR_A 5.773502f

static float quintic_ease(float tau)
{
    if (tau <= 0.0f)
        return 0.0f;
    if (tau >= 1.0f)
        return 1.0f;

    return tau * tau * tau * (10.0f + tau * (-15.0f + tau * 6.0f));
}

static float calculate_move_duration(float q0, float q1)
{
    float dist = fabsf(q1 - q0);
    if (dist < 1e-4f)
        return 0.01f;

    float t_vel = (FACTOR_V * dist) / V_MAX;
    float t_acc = sqrtf((FACTOR_A * dist) / A_MAX);

    float T = (t_vel > t_acc) ? t_vel : t_acc;
    if (T < 0.01f)
        T = 0.01f;

    return T;
}

static bool ptp_step(joint_ptp_t* p, float dt, float* q_out)
{
    if (!p->active) {
        *q_out = p->q_end;
        return true;
    }

    p->t += dt;

    float tau = p->t / p->T;
    if (tau >= 1.0f) {
        tau = 1.0f;
        p->active = false;
    }

    float s = quintic_ease(tau);
    *q_out = p->q_start + (p->q_end - p->q_start) * s;

    return !p->active;
}

static joint_ptp_t ptp;

#endif

static atlas_err_t joint_manager_notify_delta_elapsed_handler(
    joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (manager->reference.delta_time == 0.0F ||
        manager->state == ATLAS_JOINT_STATE_FAULT) {
        return ATLAS_ERR_FAIL;
    }

#ifdef JOINT_TEST

    static float32_t prev_position = 0.0F;

    if (ptp.active) {
        float q_ref;
        bool done = ptp_step(&ptp, manager->reference.delta_time, &q_ref);

        manager->reference.position = q_ref;

        if (done) {
            step_motor_set_speed(&manager->motor, 0.0f);
        }

        float32_t control_speed =
            (manager->reference.position - prev_position) / 0.05F;

        step_motor_set_speed(&manager->motor, control_speed);

        prev_position = manager->reference.position;
    }
#else

    motor_driver_err_t err =
        motor_driver_set_position(&manager->driver,
                                  manager->reference.position,
                                  manager->reference.delta_time);

    if (err != MOTOR_DRIVER_ERR_OK) {
        motor_driver_set_speed(&manager->driver,
                               0.0F,
                               manager->reference.delta_time);
        manager->state = ATLAS_JOINT_STATE_FAULT;

        return ATLAS_ERR_FAIL;
    }

    motor_driver_state_t state;
    motor_driver_get_state(&manager->driver, &state);

    ATLAS_LOG(TAG,
              "measure: %.3f, reference: %.3f, error: %.3f, control: %.3f, "
              "current: %.3f",
              state.measure_position,
              manager->reference.position,
              state.measure_position - manager->reference.position,
              state.control_speed,
              state.fault_current);

    manager->measure.position = state.measure_position;
    manager->measure.current = state.fault_current;
#endif
    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_notify_pwm_pulse_handler(
    joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    step_motor_update_step_count(&manager->motor);

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_notify_handler(joint_manager_t* manager,
                                                joint_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if ((notify & JOINT_NOTIFY_DELTA_ELAPSED) == JOINT_NOTIFY_DELTA_ELAPSED) {
        ATLAS_RET_ON_ERR(joint_manager_notify_delta_elapsed_handler(manager));
    }
    if ((notify & JOINT_NOTIFY_PWM_PULSE) == JOINT_NOTIFY_PWM_PULSE) {
        ATLAS_RET_ON_ERR(joint_manager_notify_pwm_pulse_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_start_handler(
    joint_manager_t* manager,
    joint_event_payload_start_t const* start)
{
    ATLAS_ASSERT(manager && start);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_JOINT,
                            .type = SYSTEM_EVENT_TYPE_JOINT_STARTED,
                            .payload.joint_started = {}};
    if (!joint_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_running = true;
    manager->state = ATLAS_JOINT_STATE_READY;

#ifdef JOINT_TEST
    manager->state = ATLAS_JOINT_STATE_RUNNING;
    ptp.q_start = manager->measure.position;
    ptp.q_end = manager->reference.position;
    ptp.t = 0.0f;
    ptp.T = calculate_move_duration(ptp.q_start, ptp.q_end);
    ptp.active = true;

    if (!joint_manager_start_joint_test_delta_timer(manager)) {
        return ATLAS_ERR_FAIL;
    }
#endif

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_stop_handler(
    joint_manager_t* manager,
    joint_event_payload_stop_t const* stop)
{
    ATLAS_ASSERT(manager && stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_JOINT,
                            .type = SYSTEM_EVENT_TYPE_JOINT_STOPPED,
                            .payload.joint_stopped = {}};
    if (!joint_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_running = false;
    manager->state = ATLAS_JOINT_STATE_IDLE;

    return ATLAS_ERR_OK;
}

static inline bool joint_manager_send_response(
    atlas_joint_response_t const* response)
{
    ATLAS_ASSERT(response);

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_JOINT,
                            .type = SYSTEM_EVENT_TYPE_JOINT_RESPONSE,
                            .payload.joint_response.response = *response};
    return joint_manager_send_system_event(&event);
}

static atlas_err_t joint_manager_command_get_state_handler(
    joint_manager_t* manager,
    atlas_joint_command_payload_get_state_t const* get_state)
{
    ATLAS_ASSERT(manager && get_state);
    ATLAS_LOG_FUNC(TAG);

    atlas_joint_response_t response = {
        .result = ATLAS_JOINT_RESPONSE_RESULT_FAILURE,
        .type = ATLAS_JOINT_RESPONSE_TYPE_GET_STATE};

    if (manager->state != ATLAS_JOINT_STATE_UNKNOWN) {
        response.payload.get_state.state = manager->state;
        response.result = ATLAS_JOINT_RESPONSE_RESULT_SUCCESS;
    }

    if (!joint_manager_send_response(&response)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_command_set_state_handler(
    joint_manager_t* manager,
    atlas_joint_command_payload_set_state_t const* set_state)
{
    ATLAS_ASSERT(manager && set_state);
    ATLAS_LOG_FUNC(TAG);

    atlas_joint_response_t response = {
        .result = ATLAS_JOINT_RESPONSE_RESULT_FAILURE,
        .type = ATLAS_JOINT_RESPONSE_TYPE_SET_STATE};

    if ((manager->state == ATLAS_JOINT_STATE_READY &&
         set_state->state == ATLAS_JOINT_STATE_RUNNING) ||
        (manager->state == ATLAS_JOINT_STATE_RUNNING &&
         set_state->state == ATLAS_JOINT_STATE_READY)) {
        manager->state = set_state->state;
        response.result = ATLAS_JOINT_RESPONSE_RESULT_SUCCESS;
    }

    if (!joint_manager_send_response(&response)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_command_set_reference_handler(
    joint_manager_t* manager,
    atlas_joint_command_payload_set_reference_t const* set_reference)
{
    ATLAS_ASSERT(manager && set_reference);
    ATLAS_LOG_FUNC(TAG);

    atlas_joint_response_t response = {
        .result = ATLAS_JOINT_RESPONSE_RESULT_FAILURE,
        .type = ATLAS_JOINT_RESPONSE_TYPE_SET_REFERENCE};

    if (manager->state == ATLAS_JOINT_STATE_RUNNING) {
        manager->reference = set_reference->reference;
        response.result = ATLAS_JOINT_RESPONSE_RESULT_SUCCESS;
    }

    if (!joint_manager_send_response(&response)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_command_get_measure_handler(
    joint_manager_t* manager,
    atlas_joint_command_payload_get_measure_t const* get_measure)
{
    ATLAS_ASSERT(manager && get_measure);
    ATLAS_LOG_FUNC(TAG);

    atlas_joint_response_t response = {
        .result = ATLAS_JOINT_RESPONSE_RESULT_FAILURE,
        .type = ATLAS_JOINT_RESPONSE_TYPE_GET_MEASURE};

    if (manager->state == ATLAS_JOINT_STATE_RUNNING) {
        response.payload.get_measure.measure = manager->measure;
        response.result = ATLAS_JOINT_RESPONSE_RESULT_SUCCESS;
    }

    if (!joint_manager_send_response(&response)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_command_set_parameters_handler(
    joint_manager_t* manager,
    atlas_joint_command_payload_set_parameters_t const* set_parameters)
{
    ATLAS_ASSERT(manager && set_parameters);
    ATLAS_LOG_FUNC(TAG);

    atlas_joint_response_t response = {
        .result = ATLAS_JOINT_RESPONSE_RESULT_FAILURE,
        .type = ATLAS_JOINT_RESPONSE_TYPE_SET_PARAMETERS};

    if (manager->state == ATLAS_JOINT_STATE_READY) {
        if (!joint_manager_deinitialize_drivers(manager)) {
            return ATLAS_ERR_FAIL;
        }
        manager->parameters = set_parameters->parameters;
        if (!joint_manager_initialize_drivers(manager)) {
            return ATLAS_ERR_FAIL;
        }
        response.result = ATLAS_JOINT_RESPONSE_RESULT_SUCCESS;
    }

    if (!joint_manager_send_response(&response)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_joint_command_handler(
    joint_manager_t* manager,
    joint_event_payload_joint_command_t const* joint_command)
{
    ATLAS_ASSERT(manager && joint_command);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_joint_command_t const* command = &joint_command->command;
    switch (command->type) {
        case ATLAS_JOINT_COMMAND_TYPE_GET_STATE: {
            return joint_manager_command_get_state_handler(
                manager,
                &command->payload.get_state);
        }
        case ATLAS_JOINT_COMMAND_TYPE_SET_STATE: {
            return joint_manager_command_set_state_handler(
                manager,
                &command->payload.set_state);
        }
        case ATLAS_JOINT_COMMAND_TYPE_SET_REFERENCE: {
            return joint_manager_command_set_reference_handler(
                manager,
                &command->payload.set_reference);
        }
        case ATLAS_JOINT_COMMAND_TYPE_GET_MEASURE: {
            return joint_manager_command_get_measure_handler(
                manager,
                &command->payload.get_measure);
        }
        case ATLAS_JOINT_COMMAND_TYPE_SET_PARAMETERS: {
            return joint_manager_command_set_parameters_handler(
                manager,
                &command->payload.set_parameters);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_EVENT;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_handler(joint_manager_t* manager,
                                               joint_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case JOINT_EVENT_TYPE_START: {
            return joint_manager_event_start_handler(manager,
                                                     &event->payload.start);
        }
        case JOINT_EVENT_TYPE_STOP: {
            return joint_manager_event_stop_handler(manager,
                                                    &event->payload.stop);
        }
        case JOINT_EVENT_TYPE_JOINT_COMMAND: {
            return joint_manager_event_joint_command_handler(
                manager,
                &event->payload.joint_command);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_EVENT;
        }
    }
}

atlas_err_t joint_manager_process(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    joint_notify_t notify;
    if (joint_manager_receive_joint_notify(&notify)) {
        ATLAS_LOG_ON_ERR(TAG, joint_manager_notify_handler(manager, notify));
    }

    joint_event_t event;
    while (joint_manager_has_joint_event()) {
        if (joint_manager_receive_joint_event(&event)) {
            ATLAS_LOG_ON_ERR(TAG, joint_manager_event_handler(manager, &event));
        }
    }

#ifdef USE_WATCHDOG_TASK
    if (!joint_manager_send_watchdog_notify(WATCHDOG_NOTIFY_JOINT_ALIVE)) {
        return ATLAS_ERR_FAIL;
    }
#endif

    return ATLAS_ERR_OK;
}

atlas_err_t joint_manager_initialize(joint_manager_t* manager,
                                     joint_config_t const* config,
                                     joint_parameters_t const* parameters)
{
    ATLAS_ASSERT(manager && config && parameters);

    manager->is_running = false;
    manager->state = ATLAS_JOINT_STATE_IDLE;
    manager->measure.current = 0.0F;
    manager->measure.position = 0.0F;
    manager->reference.position = 0.0F;
    manager->reference.delta_time = 0.0F;
    manager->config = *config;
    manager->parameters = *parameters;

    if (!joint_manager_initialize_drivers(manager)) {
        return ATLAS_ERR_FAIL;
    }

    if (!joint_manager_initialize_chips(manager)) {
        return ATLAS_ERR_FAIL;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_JOINT,
                            .type = SYSTEM_EVENT_TYPE_JOINT_READY,
                            .payload.joint_ready = {}};
    if (!joint_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

#ifdef USE_WATCHDOG_TASK
    if (!joint_manager_send_watchdog_notify(WATCHDOG_NOTIFY_JOINT_ALIVE)) {
        return ATLAS_ERR_FAIL;
    }
#endif

    return ATLAS_ERR_OK;
}
