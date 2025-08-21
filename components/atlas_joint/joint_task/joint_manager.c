#include "joint_manager.h"
#include "FreeRTOS.h"
#include "a4988.h"
#include "common.h"
#include "event.h"
#include "manager.h"
#include "motor_driver.h"
#include "notify.h"
#include "pid_regulator.h"
#include "step_motor.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

static bool frequency_to_prescaler_and_period(uint32_t frequency_hz,
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

static a4988_err_t a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    joint_config_t* config = (joint_config_t*)user;

    HAL_GPIO_WritePin(config->a4988_gpio, pin, (GPIO_PinState)state);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_start(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    HAL_TIM_PWM_Start_IT(config->a4988_pwm_timer, config->a4988_pwm_channel);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_stop(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    HAL_TIM_PWM_Stop_IT(config->a4988_pwm_timer, config->a4988_pwm_channel);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_set_frequency(void* user, uint32_t frequency)
{
    joint_config_t* config = (joint_config_t*)user;

    uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        clock_hz *= 2;
    }

    uint32_t prescaler;
    uint32_t period;
    bool result = frequency_to_prescaler_and_period(frequency,
                                                    clock_hz,
                                                    0xFFFFU,
                                                    0xFFFFU,
                                                    &prescaler,
                                                    &period);

    if (result && period < 0xFFFFU && prescaler < 0xFFFFU) {
        uint32_t tick_hz = clock_hz / (prescaler + 1);
        uint32_t compare = (tick_hz / 1000000) * 5; // 5 us pulse
        if (compare == 0) {
            compare = 1;
        }
        if (compare > period) {
            compare = period;
        }

        __HAL_TIM_DISABLE(config->a4988_pwm_timer);
        __HAL_TIM_SET_COUNTER(config->a4988_pwm_timer, 0U);
        __HAL_TIM_SET_PRESCALER(config->a4988_pwm_timer, prescaler);
        __HAL_TIM_SET_AUTORELOAD(config->a4988_pwm_timer, period);
        __HAL_TIM_SET_COMPARE(config->a4988_pwm_timer,
                              config->a4988_pwm_channel,
                              compare);
        __HAL_TIM_ENABLE(config->a4988_pwm_timer);
    }

    return A4988_ERR_OK;
}

static as5600_err_t as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    joint_config_t* config = (joint_config_t*)user;

    HAL_GPIO_WritePin(config->as5600_gpio, pin, (GPIO_PinState)state);

    return AS5600_ERR_OK;
}

static as5600_err_t as5600_bus_write_data(void* user,
                                          uint8_t address,
                                          uint8_t const* data,
                                          size_t data_size)
{
    joint_config_t* config = (joint_config_t*)user;

    SemaphoreHandle_t joint_mutex = semaphore_manager_get(SEMAPHORE_TYPE_JOINT);

    if (xSemaphoreTake(joint_mutex, pdMS_TO_TICKS(1))) {
        HAL_I2C_Mem_Write(config->as5600_i2c_bus,
                          config->as5600_i2c_address << 1,
                          address,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          data_size,
                          100);
        xSemaphoreGive(joint_mutex);
    }

    return AS5600_ERR_OK;
}

static as5600_err_t as5600_bus_read_data(void* user,
                                         uint8_t address,
                                         uint8_t* data,
                                         size_t data_size)
{
    joint_config_t* config = (joint_config_t*)user;

    SemaphoreHandle_t joint_mutex = semaphore_manager_get(SEMAPHORE_TYPE_JOINT);

    if (xSemaphoreTake(joint_mutex, pdMS_TO_TICKS(1))) {
        HAL_I2C_Mem_Read(config->as5600_i2c_bus,
                         config->as5600_i2c_address << 1,
                         address,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         data_size,
                         100);
        xSemaphoreGive(joint_mutex);
    }

    return AS5600_ERR_OK;
}

static as5600_err_t as5600_initialize_chip(as5600_t* as5600,
                                           float32_t min_angle,
                                           float32_t max_angle)
{
    as5600_status_reg_t status;
    as5600_err_t err = as5600_get_status_reg(as5600, &status);
    if (err != AS5600_ERR_OK)
        return err;

    float32_t angle_range = (max_angle - min_angle);

    uint16_t min_raw = (uint16_t)(min_angle / angle_range * 4095.0F);
    uint16_t max_raw = (uint16_t)(max_angle / angle_range * 4095.0F);

    as5600_zpos_reg_t zpos = {.zpos = min_raw & 0x0FFF};
    err = as5600_set_zpos_reg(as5600, &zpos);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_mpos_reg_t mpos = {.mpos = max_raw & 0x0FFF};
    err = as5600_set_mpos_reg(as5600, &mpos);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_conf_reg_t conf = {.wd = AS5600_WATCHDOG_OFF,
                              .fth = AS5600_SLOW_FILTER_X16,
                              .sf = AS5600_SLOW_FILTER_X16,
                              .pwmf = AS5600_PWM_FREQUENCY_115HZ,
                              .outs = AS5600_FAST_FILTER_THRESH_SLOW,
                              .hyst = AS5600_HYSTERESIS_OFF,
                              .pm = AS5600_POWER_MODE_NOM};
    err = as5600_set_conf_reg(as5600, &conf);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_zmco_reg_t zmco;
    err = as5600_get_zmco_reg(as5600, &zmco);
    if (err != AS5600_ERR_OK)
        return err;

    return AS5600_ERR_OK;
}

static ina226_err_t ina226_bus_write_data(void* user,
                                          uint8_t address,
                                          uint8_t const* data,
                                          size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_config_t* config = (joint_config_t*)user;

    SemaphoreHandle_t joint_mutex = semaphore_manager_get(SEMAPHORE_TYPE_JOINT);

    if (xSemaphoreTake(joint_mutex, pdMS_TO_TICKS(1))) {
        HAL_I2C_Mem_Write(config->ina226_i2c_bus,
                          config->ina226_i2c_address,
                          address,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t*)data,
                          data_size,
                          10);
        xSemaphoreGive(joint_mutex);
    }

    return INA226_ERR_OK;
}

static ina226_err_t ina226_bus_read_data(void* user,
                                         uint8_t address,
                                         uint8_t* data,
                                         size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_config_t* config = (joint_config_t*)user;

    SemaphoreHandle_t joint_mutex = semaphore_manager_get(SEMAPHORE_TYPE_JOINT);

    if (xSemaphoreTake(joint_mutex, pdMS_TO_TICKS(1))) {
        HAL_I2C_Mem_Read(config->ina226_i2c_bus,
                         config->ina226_i2c_address,
                         address,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         data_size,
                         10);
        xSemaphoreGive(joint_mutex);
    }

    return INA226_ERR_OK;
}

static ina226_err_t ina226_initialize_chip(ina226_t* ina226,
                                           float32_t min_current,
                                           float32_t max_current)
{
    return INA226_ERR_OK;
}

static step_motor_err_t step_motor_device_set_frequency(void* user,
                                                        uint32_t frequency)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_frequency(&manager->a4988, frequency);

    return STEP_MOTOR_ERR_OK;
}

static step_motor_err_t step_motor_device_set_direction(
    void* user,
    step_motor_direction_t direction)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    a4988_set_direction(&manager->a4988, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

static motor_driver_err_t motor_driver_motor_set_speed(void* user,
                                                       float32_t speed)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    step_motor_set_speed(&manager->motor, speed);

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_encoder_get_position(void* user,
                                                            float32_t* position)
{
    ATLAS_ASSERT(user && position);

    joint_manager_t* manager = (joint_manager_t*)user;

    as5600_get_angle_data_scaled_bus(&manager->as5600, position);

    manager->measure.position = *position;

    return MOTOR_DRIVER_ERR_OK;
}

static motor_driver_err_t motor_driver_regulator_get_control(
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

static motor_driver_err_t motor_driver_fault_get_current(void* user,
                                                         float32_t* current)
{
    ATLAS_ASSERT(user && current);

    joint_manager_t* manager = (joint_manager_t*)user;

    manager->measure.current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}

static char const* const TAG = "joint_manager";

static inline bool joint_manager_has_joint_event(void)
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_JOINT)) ==
           pdPASS;
}

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

static atlas_err_t joint_manager_notify_delta_timer_handler(
    joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (manager->reference.delta_time == 0.0F || manager->has_fault) {
        return ATLAS_ERR_FAIL;
    }

    motor_driver_err_t err =
        motor_driver_set_position(&manager->driver,
                                  manager->reference.position,
                                  manager->reference.delta_time);
   
    ATLAS_LOG(TAG, "set position result: %d", err);

    if (err != MOTOR_DRIVER_ERR_OK) {
        // if (!joint_manager_send_system_notify(SYSTEM_NOTIFY_JOINT_FAULT)) {
        //     return ATLAS_ERR_FAIL;
        // }

        step_motor_set_speed(&manager->motor, 0.0F);
        manager->has_fault = true;
    } else {
        system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_JOINT};
        event.type = SYSTEM_EVENT_TYPE_JOINT_MEASURE;
        event.payload.joint_measure = manager->measure;

        if (!joint_manager_send_system_event(&event)) {
            return ATLAS_ERR_FAIL;
        }

        if (manager->has_fault) {
            manager->has_fault = false;
        }
    }

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

    if (notify & JOINT_NOTIFY_DELTA_TIMER) {
        ATLAS_RET_ON_ERR(joint_manager_notify_delta_timer_handler(manager));
    }
    if (notify & JOINT_NOTIFY_PWM_PULSE) {
        ATLAS_RET_ON_ERR(joint_manager_notify_pwm_pulse_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_start_handler(
    joint_manager_t* manager,
    joint_event_payload_start_t const* payload)
{
    ATLAS_ASSERT(manager && payload);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    if (manager->has_fault) {
        return ATLAS_ERR_IMPROPER_STATE;
    }

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_stop_handler(
    joint_manager_t* manager,
    joint_event_payload_stop_t const* payload)
{
    ATLAS_ASSERT(manager && payload);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    step_motor_device_set_direction(&manager->motor, STEP_MOTOR_DIRECTION_STOP);

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_reset_handler(
    joint_manager_t* manager,
    joint_event_payload_reset_t const* payload)
{
    ATLAS_ASSERT(manager && payload);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    pid_regulator_reset(&manager->regulator);
    step_motor_reset(&manager->motor);

    manager->is_running = false;
    manager->has_fault = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_reference_handler(
    joint_manager_t* manager,
    joint_event_payload_reference_t const* reference)
{
    ATLAS_ASSERT(manager && reference);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_joint_reference_print(reference);

    manager->reference = *reference;

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
        case JOINT_EVENT_TYPE_REFERENCE: {
            return joint_manager_event_reference_handler(
                manager,
                &event->payload.reference);
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

    return ATLAS_ERR_OK;
}

atlas_err_t joint_manager_initialize(joint_manager_t* manager,
                                     joint_config_t const* config,
                                     joint_parameters_t const* parameters)
{
    ATLAS_ASSERT(manager && config && parameters);

    manager->config = *config;
    manager->is_running = false;
    manager->has_fault = false;
    manager->measure.current = 0.0F;
    manager->measure.position = 0.0F;
    manager->reference.position = 0.0F;
    manager->reference.delta_time = 0.0F;

    as5600_initialize(
        &manager->as5600,
        &(as5600_config_t){.dir_pin = config->as5600_dir_pin,
                           .max_angle = parameters->max_position,
                           .min_angle = parameters->min_position},
        &(as5600_interface_t){.gpio_user = &manager->config,
                              .gpio_write_pin = as5600_gpio_write_pin,
                              .bus_user = &manager->config,
                              .bus_read_data = as5600_bus_read_data,
                              .bus_write_data = as5600_bus_write_data});

    as5600_initialize_chip(&manager->as5600,
                           parameters->min_position,
                           parameters->max_position);

    a4988_initialize(
        &manager->a4988,
        &(a4988_config_t){.pin_dir = config->a4988_dir_pin},
        &(a4988_interface_t){.gpio_user = &manager->config,
                             .gpio_write_pin = a4988_gpio_write_pin,
                             .pwm_user = &manager->config,
                             .pwm_start = a4988_pwm_start,
                             .pwm_stop = a4988_pwm_stop,
                             .pwm_set_frequency = a4988_pwm_set_frequency});

    step_motor_initialize(
        &manager->motor,
        &(step_motor_config_t){.min_position = parameters->min_position,
                               .max_position = parameters->max_position,
                               .min_speed = parameters->min_speed,
                               .max_speed = parameters->max_speed,
                               .step_change = parameters->step_change},
        &(step_motor_interface_t){
            .device_user = manager,
            .device_set_frequency = step_motor_device_set_frequency,
            .device_set_direction = step_motor_device_set_direction},
        0.0F);

    pid_regulator_initialize(&manager->regulator,
                             &(pid_regulator_config_t){
                                 .prop_gain = parameters->prop_gain,
                                 .int_gain = parameters->int_gain,
                                 .dot_gain = parameters->dot_gain,
                                 .sat_gain = parameters->sat_gain,
                                 .min_control = parameters->min_speed,
                                 .max_control = parameters->max_speed,
                                 .dead_error = parameters->step_change / 2.0F});

    motor_driver_initialize(
        &manager->driver,
        &(motor_driver_config_t){.min_position = parameters->min_position,
                                 .max_position = parameters->max_position,
                                 .min_speed = parameters->min_speed,
                                 .max_speed = parameters->max_speed,
                                 .max_current = parameters->current_limit},
        &(motor_driver_interface_t){
            .motor_user = manager,
            .motor_set_speed = motor_driver_motor_set_speed,
            .encoder_user = manager,
            .encoder_get_position = motor_driver_encoder_get_position,
            .regulator_user = manager,
            .regulator_get_control = motor_driver_regulator_get_control,
            .fault_user = manager,
            .fault_get_current = motor_driver_fault_get_current});

    if (!joint_manager_send_system_notify(SYSTEM_NOTIFY_JOINT_READY)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}
