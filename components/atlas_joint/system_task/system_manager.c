#include "system_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_rtc.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

static char const* const TAG = "system_manager";

static inline bool system_manager_has_system_event(void)
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_SYSTEM));
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

static inline bool system_manager_receive_system_notify(system_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0,
                           SYSTEM_NOTIFY_ALL,
                           (uint32_t*)notify,
                           pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_start_retry_timer(void)
{
    xTimerStart(timer_manager_get(TIMER_TYPE_SYSTEM), pdMS_TO_TICKS(1)) ==
        pdPASS;
}

static inline bool system_manager_stop_retry_timer(void)
{
    return xTimerStop(timer_manager_get(TIMER_TYPE_SYSTEM), pdMS_TO_TICKS(1)) ==
           pdPASS;
}

static inline bool system_manager_get_rtc_timestamp(
    system_manager_t* manager,
    atlas_timestamp_t* timestamp)
{
    ATLAS_ASSERT(manager && timestamp);

    RTC_TimeTypeDef rtc_time;
    if (HAL_RTC_GetTime(manager->config.timestamp_rtc,
                        &rtc_time,
                        RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }

    RTC_DateTypeDef rtc_date;
    if (HAL_RTC_GetDate(manager->config.timestamp_rtc,
                        &rtc_date,
                        RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }

    return true;
}

static atlas_err_t system_manager_notify_retry_timer_handler(
    system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    inline bool (*retry_timer_function)(void) =
        system_manager_start_retry_timer;

    if (manager->is_packet_running) {
        packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINT_READY};
        event.payload.joint_ready = (packet_event_payload_joint_ready_t){};

        if (!system_manager_send_packet_event(&event)) {
            return ATLAS_ERR_FAIL;
        }

        retry_timer_function = system_manager_stop_retry_timer;
    }

    if (!retry_timer_function()) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_packet_ready_handler(
    system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    packet_event_t event = {.type = PACKET_EVENT_TYPE_START};
    event.payload.start = (packet_event_payload_start_t){};

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_packet_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_joint_ready_handler(
    system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (manager->is_packet_running) {
        packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINT_READY};
        event.payload.joint_ready = (packet_event_payload_joint_ready_t){};

        if (!system_manager_send_packet_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    } else {
        if (!system_manager_start_retry_timer()) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_joint_fault_handler(
    system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINT_FAULT};
    event.payload.joint_fault = (packet_event_payload_joint_fault_t){};

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_handler(system_manager_t* manager,
                                                 system_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & SYSTEM_NOTIFY_RETRY_TIMER) {
        ATLAS_RET_ON_ERR(system_manager_notify_retry_timer_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_JOINT_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_joint_ready_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_JOINT_FAULT) {
        ATLAS_RET_ON_ERR(system_manager_notify_joint_fault_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_PACKET_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_packet_ready_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_start_handler(
    system_manager_t* manager,
    system_event_payload_joint_start_t const* joint_start)
{
    ATLAS_ASSERT(manager && joint_start);
    ATLAS_LOG_FUNC(TAG);

    joint_event_t event = {.type = JOINT_EVENT_TYPE_START};
    event.payload.start = (joint_event_payload_start_t){};

    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    system_manager_get_rtc_timestamp(manager, &manager->start_timestamp);

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_stop_handler(
    system_manager_t* manager,
    system_event_payload_joint_stop_t const* joint_stop)
{
    ATLAS_ASSERT(manager && joint_stop);
    ATLAS_LOG_FUNC(TAG);

    joint_event_t event = {.type = JOINT_EVENT_TYPE_STOP};
    event.payload.stop = (joint_event_payload_stop_t){};

    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    memset(&manager->start_timestamp, 0, sizeof(manager->start_timestamp));

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_measured_joint_data_handler(
    system_manager_t* manager,
    system_event_payload_joint_data_t const* joint_data)
{
    ATLAS_ASSERT(manager && joint_data);
    ATLAS_LOG_FUNC(TAG);

    if (manager->measured_position == joint_data->position) {
        return ATLAS_ERR_OK;
    }

    manager->measured_position = joint_data->position;

    packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINT_DATA};
    event.payload.joint_data.num = manager->config.num;
    event.payload.joint_data.data.position = manager->measured_position;
    event.payload.joint_data.timestamp = manager->current_timestamp;

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_referenced_joint_data_handler(
    system_manager_t* manager,
    system_event_payload_joint_data_t const* joint_data)
{
    ATLAS_ASSERT(manager && joint_data);
    ATLAS_LOG_FUNC(TAG);

    if (manager->measured_position == joint_data->position) {
        return ATLAS_ERR_OK;
    }

    manager->referenced_position = joint_data->position;

    joint_event_t event = {.type = JOINT_EVENT_TYPE_JOINT_DATA};
    event.payload.joint_data.position = manager->referenced_position;

    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_handler(system_manager_t* manager,
                                                system_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case SYSTEM_EVENT_TYPE_JOINT_START: {
            return system_manager_event_joint_start_handler(
                manager,
                &event->payload.joint_start);
        }
        case SYSTEM_EVENT_TYPE_JOINT_STOP: {
            return system_manager_event_joint_stop_handler(
                manager,
                &event->payload.joint_stop);
        }
        case SYSTEM_EVENT_TYPE_JOINT_DATA: {
            switch (event->origin) {
                case SYSTEM_EVENT_ORIGIN_JOINT: {
                    return system_manager_event_measured_joint_data_handler(
                        manager,
                        &event->payload.joint_data);
                }
                case SYSTEM_EVENT_ORIGIN_PACKET: {
                    return system_manager_event_referenced_joint_data_handler(
                        manager,
                        &event->payload.joint_data);
                }
                default: {
                    return ATLAS_ERR_UNKNOWN_ORIGIN;
                }
            }
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
        ATLAS_RET_ON_ERR(system_manager_notify_handler(manager, notify));
    }

    system_event_t event;
    while (system_manager_has_system_event()) {
        if (system_manager_receive_system_event(&event)) {
            ATLAS_RET_ON_ERR(system_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t system_manager_initialize(system_manager_t* manager,
                                      system_config_t const* config)
{
    ATLAS_ASSERT(manager && config);

    manager->is_running = true;
    manager->is_packet_running = false;
    manager->config = *config;

    return ATLAS_ERR_OK;
}
