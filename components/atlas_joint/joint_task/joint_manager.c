#include "joint_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "drv8825.h"
#include "event.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "step_motor.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

drv8825_err_t joint_drv8825_gpio_write_pin(void* user, uint32_t pin, bool state);
drv8825_err_t joint_drv8825_pwm_start(void* user);
drv8825_err_t joint_drv8825_pwm_stop(void* user);
drv8825_err_t joint_drv8825_pwm_set_frequency(void* user, uint32_t frequency);

as5600_err_t joint_as5600_gpio_write_pin(void* user, uint32_t pin, bool state);
as5600_err_t joint_as5600_bus_write_data(void* user,
                                         uint8_t address,
                                         uint8_t const* data,
                                         size_t data_size);
as5600_err_t joint_as5600_bus_read_data(void* user,
                                        uint8_t address,
                                        uint8_t* data,
                                        size_t data_size);

ina226_err_t joint_ina226_bus_write_data(void* user,
                                         uint8_t address,
                                         uint8_t const* data,
                                         size_t data_size);
ina226_err_t joint_ina226_bus_read_data(void* user,
                                        uint8_t address,
                                        uint8_t* data,
                                        size_t data_size);

step_motor_err_t joint_step_motor_device_set_frequency(void* user, uint32_t frequency);
step_motor_err_t joint_step_motor_device_set_direction(void* user,
                                                       step_motor_direction_t direction);

motor_driver_err_t joint_motor_driver_motor_set_speed(void* user, float32_t speed);
motor_driver_err_t joint_motor_driver_encoder_get_position(void* user, float32_t* position);
motor_driver_err_t joint_motor_driver_regulator_get_control(void* user,
                                                            float32_t error,
                                                            float32_t* control,
                                                            float32_t delta_time);
motor_driver_err_t joint_motor_driver_fault_get_current(void* user, float32_t* current);

static char const* const TAG = "joint_manager";

static inline bool joint_manager_has_joint_event(joint_manager_t const* manager)
{
    ATLAS_ASSERT(manager);

    return uxQueueMessagesWaiting(manager->queue) == pdPASS;
}

static inline bool joint_manager_send_joints_notify(joints_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_JOINTS), (uint32_t)notify, eSetBits) == pdPASS;
}

static inline bool joint_manager_send_joints_event(joints_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_JOINTS), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joint_manager_receive_joint_event(joint_manager_t const* manager,
                                                     joint_event_t* event)
{
    ATLAS_ASSERT(manager && event);

    return xQueueReceive(manager->queue, event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool joint_manager_receive_joint_notify(joint_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, JOINT_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static atlas_err_t joint_manager_notify_delta_timer_handler(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    float32_t measured_position;
    motor_driver_set_position(&manager->driver,
                              manager->goal_position,
                              manager->delta_time,
                              &measured_position);

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_JOINT,
                            .type = SYSTEM_EVENT_TYPE_JOINT_DATA};
    event.payload.measure_data.position = measured_position;
    event.payload.measure_data.num = manager->num;

    if (!joint_manager_send_joints_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_notify_pwm_pulse_handler(joint_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    step_motor_update_step_count(&manager->motor);

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_notify_handler(joint_manager_t* manager, joint_notify_t notify)
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

static atlas_err_t joint_manager_event_start_handler(joint_manager_t* manager,
                                                     joint_event_payload_start_t const* payload)
{
    ATLAS_ASSERT(manager && payload);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    step_motor_reset(&manager->motor);

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_stop_handler(joint_manager_t* manager,
                                                    joint_event_payload_stop_t const* payload)
{
    ATLAS_ASSERT(manager && payload);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    step_motor_reset(&manager->motor);

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_position_handler(
    joint_manager_t* manager,
    joint_event_payload_position_t const* position)
{
    ATLAS_ASSERT(manager && position);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    ATLAS_LOG(TAG,
              "joint_num: %d, reference_position: %d * 100",
              manager->num,
              (int32_t)*position * 100);

    manager->goal_position = *position;

    return ATLAS_ERR_OK;
}

static atlas_err_t joint_manager_event_handler(joint_manager_t* manager, joint_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case JOINT_EVENT_TYPE_START: {
            return joint_manager_event_start_handler(manager, &event->payload.start);
        }
        case JOINT_EVENT_TYPE_STOP: {
            return joint_manager_event_stop_handler(manager, &event->payload.stop);
        }
        case JOINT_EVENT_TYPE_POSITION: {
            return joint_manager_event_position_handler(manager, &event->payload.position);
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
        ATLAS_RET_ON_ERR(joint_manager_notify_handler(manager, notify));
    }

    joint_event_t event;
    while (joint_manager_has_joint_event(manager)) {
        if (joint_manager_receive_joint_event(manager, &event)) {
            ATLAS_RET_ON_ERR(joint_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t joint_manager_initialize(joint_manager_t* manager,
                                     joint_num_t num,
                                     QueueHandle_t queue,
                                     joint_config_t const* config,
                                     joint_parameters_t const* parameters)
{
    ATLAS_ASSERT(manager && config && parameters && queue);

    manager->is_running = false;
    manager->queue = queue;
    manager->config = *config;
    manager->num = num;
    manager->delta_time = JOINTS_DELTA_TIMER_PERIOD_S;

    as5600_initialize(&manager->as5600,
                      &(as5600_config_t){.dir_pin = config->as5600_dir_pin,
                                         .max_angle = parameters->max_position,
                                         .min_angle = parameters->min_position},
                      &(as5600_interface_t){.gpio_user = &manager->config,
                                            .gpio_write_pin = joint_as5600_gpio_write_pin,
                                            .bus_user = &manager->config,
                                            .bus_read_data = joint_as5600_bus_read_data,
                                            .bus_write_data = joint_as5600_bus_write_data});

    drv8825_initialize(
        &manager->drv8825,
        &(drv8825_config_t){.pin_dir = config->drv8825_dir_pin},
        &(drv8825_interface_t){.gpio_user = &manager->config,
                               .gpio_write_pin = joint_drv8825_gpio_write_pin,
                               .pwm_user = &manager->config,
                               .pwm_start = joint_drv8825_pwm_start,
                               .pwm_stop = joint_drv8825_pwm_stop,
                               .pwm_set_frequency = joint_drv8825_pwm_set_frequency});

    step_motor_initialize(
        &manager->motor,
        &(step_motor_config_t){.min_position = parameters->min_position,
                               .max_position = parameters->max_position,
                               .min_speed = parameters->min_speed,
                               .max_speed = parameters->max_speed,
                               .step_change = parameters->step_change},
        &(step_motor_interface_t){.device_user = manager,
                                  .device_set_frequency = joint_step_motor_device_set_frequency,
                                  .device_set_direction = joint_step_motor_device_set_direction},
        0.0F);

    pid_regulator_initialize(&manager->regulator,
                             &(pid_regulator_config_t){.prop_gain = parameters->prop_gain,
                                                       .int_gain = parameters->int_gain,
                                                       .dot_gain = parameters->dot_gain,
                                                       .sat_gain = parameters->sat_gain,
                                                       .min_control = parameters->min_speed,
                                                       .max_control = parameters->max_speed});

    motor_driver_initialize(&manager->driver,
                            &(motor_driver_config_t){.min_position = parameters->min_position,
                                                     .max_position = parameters->max_position,
                                                     .min_speed = parameters->min_speed,
                                                     .max_speed = parameters->max_speed,
                                                     .max_current = parameters->current_limit},
                            &(motor_driver_interface_t){
                                .motor_user = manager,
                                .motor_set_speed = joint_motor_driver_motor_set_speed,
                                .encoder_user = manager,
                                .encoder_get_position = joint_motor_driver_encoder_get_position,
                                .regulator_user = manager,
                                .regulator_get_control = joint_motor_driver_regulator_get_control,
                                .fault_user = manager,
                                .fault_get_current = joint_motor_driver_fault_get_current});

    if (!joint_manager_send_joints_notify(1 << num)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}
