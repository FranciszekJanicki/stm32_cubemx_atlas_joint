#include "system_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rtc.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

static char const* const TAG = "atlas_joint:system_manager";

static inline bool system_manager_get_delta_timer_pin(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return (bool)HAL_GPIO_ReadPin(manager->config.delta_timer_gpio,
                                  manager->config.delta_timer_pin);
}

static inline bool system_manager_has_system_event(void)
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_SYSTEM)) > 0U;
}

static inline bool system_manager_send_joint_event(joint_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_JOINT),
                      event,
                      pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_send_packet_event(packet_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_PACKET),
                      event,
                      pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_receive_system_event(system_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_SYSTEM),
                         event,
                         pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_send_log_notify(log_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_LOG), notify, eSetBits) ==
           pdPASS;
}

static inline bool system_manager_receive_system_notify(system_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0,
                           SYSTEM_NOTIFY_ALL,
                           (uint32_t*)notify,
                           pdMS_TO_TICKS(1)) == pdPASS;
}

static atlas_err_t system_manager_notify_handler(system_manager_t* manager,
                                                 system_notify_t notify)
{
    ATLAS_ASSERT(manager);

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_packet_ready_handler(
    system_manager_t* manager,
    system_event_payload_packet_ready_t const* packet_ready)
{
    ATLAS_ASSERT(manager && packet_ready);
    ATLAS_LOG_FUNC(TAG);

    packet_event_t event = {.type = PACKET_EVENT_TYPE_START,
                            .payload.start = {}};
    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_packet_started_handler(
    system_manager_t* manager,
    system_event_payload_packet_started_t const* packet_started)
{
    ATLAS_ASSERT(manager && packet_started);
    ATLAS_LOG_FUNC(TAG);

    manager->is_packet_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_packet_stopped_handler(
    system_manager_t* manager,
    system_event_payload_packet_stopped_t const* packet_stopped)
{
    ATLAS_ASSERT(manager && packet_stopped);
    ATLAS_LOG_FUNC(TAG);

    manager->is_packet_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_command_handler(
    system_manager_t* manager,
    system_event_payload_joint_command_t const* joint_command)
{
    ATLAS_ASSERT(manager && joint_command);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_joint_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_JOINT_COMMAND,
                           .payload.joint_command.command =
                               joint_command->command};
    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_ready_handler(
    system_manager_t* manager,
    system_event_payload_joint_ready_t const* joint_ready)
{
    ATLAS_ASSERT(manager && joint_ready);
    ATLAS_LOG_FUNC(TAG);

    joint_event_t event = {.type = JOINT_EVENT_TYPE_START, .payload.start = {}};
    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_started_handler(
    system_manager_t* manager,
    system_event_payload_joint_started_t const* joint_started)
{
    ATLAS_ASSERT(manager && joint_started);
    ATLAS_LOG_FUNC(TAG);

    manager->is_joint_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_stopped_handler(
    system_manager_t* manager,
    system_event_payload_joint_stopped_t const* joint_stopped)
{
    ATLAS_ASSERT(manager && joint_stopped);
    ATLAS_LOG_FUNC(TAG);

    manager->is_joint_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_response_handler(
    system_manager_t* manager,
    system_event_payload_joint_response_t const* joint_response)
{
    ATLAS_ASSERT(manager && joint_response);
    ATLAS_LOG_FUNC(TAG);

    packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINT_RESPONSE,
                            .payload.joint_response = joint_response->response};
    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_handler(system_manager_t* manager,
                                                system_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case SYSTEM_EVENT_TYPE_PACKET_READY: {
            return system_manager_event_packet_ready_handler(
                manager,
                &event->payload.packet_ready);
        }
        case SYSTEM_EVENT_TYPE_PACKET_STARTED: {
            return system_manager_event_packet_started_handler(
                manager,
                &event->payload.packet_started);
        }
        case SYSTEM_EVENT_TYPE_PACKET_STOPPED: {
            return system_manager_event_packet_stopped_handler(
                manager,
                &event->payload.packet_stopped);
        }
        case SYSTEM_EVENT_TYPE_JOINT_COMMAND: {
            return system_manager_event_joint_command_handler(
                manager,
                &event->payload.joint_command);
        }
        case SYSTEM_EVENT_TYPE_JOINT_READY: {
            return system_manager_event_joint_ready_handler(
                manager,
                &event->payload.joint_ready);
        }
        case SYSTEM_EVENT_TYPE_JOINT_STARTED: {
            return system_manager_event_joint_started_handler(
                manager,
                &event->payload.joint_started);
        }
        case SYSTEM_EVENT_TYPE_JOINT_STOPPED: {
            return system_manager_event_joint_stopped_handler(
                manager,
                &event->payload.joint_stopped);
        }
        case SYSTEM_EVENT_TYPE_JOINT_RESPONSE: {
            return system_manager_event_joint_response_handler(
                manager,
                &event->payload.joint_response);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_EVENT;
        }
    }
}

atlas_err_t system_manager_process(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    system_notify_t notify;
    if (system_manager_receive_system_notify(&notify)) {
        ATLAS_LOG_ON_ERR(TAG, system_manager_notify_handler(manager, notify));
    }

    system_event_t event;
    while (system_manager_has_system_event()) {
        if (system_manager_receive_system_event(&event)) {
            ATLAS_LOG_ON_ERR(TAG,
                             system_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t system_manager_initialize(system_manager_t* manager,
                                      system_config_t const* config)
{
    ATLAS_ASSERT(manager && config);

    manager->is_packet_running = false;
    manager->is_joint_running = false;

    memcpy(&manager->config, config, sizeof(*config));

#ifdef USE_LOG_TASK
    if (!system_manager_send_log_notify(LOG_NOTIFY_START)) {
        return ATLAS_ERR_FAIL;
    }
#endif

    return ATLAS_ERR_OK;
}
