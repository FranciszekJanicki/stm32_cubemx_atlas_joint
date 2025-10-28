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

static inline bool system_manager_set_rtc_timestamp(
    system_manager_t* manager,
    atlas_timestamp_t const* timestamp)
{
    ATLAS_ASSERT(manager && timestamp);

    RTC_TimeTypeDef rtc_time;

    rtc_time.Hours = timestamp->hour;
    rtc_time.Minutes = timestamp->minute;
    rtc_time.Seconds = timestamp->second;

    if (HAL_RTC_SetTime(manager->config.timestamp_rtc,
                        &rtc_time,
                        RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }

    RTC_DateTypeDef rtc_date;

    rtc_date.Year = timestamp->year;
    rtc_date.Month = timestamp->month;
    rtc_date.Date = timestamp->day;

    if (HAL_RTC_SetDate(manager->config.timestamp_rtc,
                        &rtc_date,
                        RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }

    return true;
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

    timestamp->hour = rtc_time.Hours;
    timestamp->minute = rtc_time.Minutes;
    timestamp->second = rtc_time.Seconds;

    RTC_DateTypeDef rtc_date;
    if (HAL_RTC_GetDate(manager->config.timestamp_rtc,
                        &rtc_date,
                        RTC_FORMAT_BIN) != HAL_OK) {
        return false;
    }

    timestamp->year = rtc_date.Year;
    timestamp->month = rtc_date.Month;
    timestamp->day = rtc_date.Date;

    return true;
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

    manager->is_packet_ready = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_packet_started_handler(
    system_manager_t* manager,
    system_event_payload_packet_started_t const* packet_started)
{
    ATLAS_ASSERT(manager && packet_started);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_joint_ready_pending) {
        packet_event_t event = {
            .type = PACKET_EVENT_TYPE_JOINT_READY,
            .payload.joint_ready = {.ready = {},
                                    .num = manager->config.joint_num,
                                    .timestamp = manager->current_timestamp}};
        if (!system_manager_send_packet_event(&event)) {
            return ATLAS_ERR_FAIL;
        }

        manager->is_joint_ready_pending = false;
    }

    if (manager->is_joint_fault_pending) {
        packet_event_t event = {
            .type = PACKET_EVENT_TYPE_JOINT_FAULT,
            .payload.joint_fault = {.fault = {},
                                    .num = manager->config.joint_num,
                                    .timestamp = manager->current_timestamp}};
        if (!system_manager_send_packet_event(&event)) {
            return ATLAS_ERR_FAIL;
        }

        manager->is_joint_fault_pending = false;
    }

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

static atlas_err_t system_manager_event_joint_start_handler(
    system_manager_t* manager,
    system_event_payload_joint_start_t const* joint_start)
{
    ATLAS_ASSERT(manager && joint_start);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_joint_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    if (manager->is_joint_ready) {
        joint_event_t event = {.type = JOINT_EVENT_TYPE_START,
                               .payload.start = {.start = joint_start->start}};
        if (!system_manager_send_joint_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    } else {
        manager->is_joint_start_pending = true;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_stop_handler(
    system_manager_t* manager,
    system_event_payload_joint_stop_t const* joint_stop)
{
    ATLAS_ASSERT(manager && joint_stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_joint_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_STOP,
                           .payload.stop = {.stop = joint_stop->stop}};
    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_reset_handler(
    system_manager_t* manager,
    system_event_payload_joint_reset_t const* joint_reset)
{
    ATLAS_ASSERT(manager && joint_reset);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_joint_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_RESET,
                           .payload.reset = {.reset = joint_reset->reset}};
    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_reference_handler(
    system_manager_t* manager,
    system_event_payload_joint_reference_t const* joint_reference)
{
    ATLAS_ASSERT(manager && joint_reference);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_joint_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (atlas_joint_reference_is_equal(&manager->joint_reference,
                                       &joint_reference->reference)) {
        return ATLAS_ERR_OK;
    }

    joint_event_t event = {
        .type = JOINT_EVENT_TYPE_REFERENCE,
        .payload.reference = {.reference = joint_reference->reference}};
    if (!system_manager_send_joint_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->joint_reference = joint_reference->reference;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_parameters_handler(
    system_manager_t* manager,
    system_event_payload_joint_parameters_t const* joint_parameters)
{
    ATLAS_ASSERT(manager && joint_parameters);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_joint_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {
        .type = JOINT_EVENT_TYPE_PARAMETERS,
        .payload.parameters = {.parameters = joint_parameters->parameters}};
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

    if (manager->is_joint_start_pending) {
        joint_event_t event = {.type = JOINT_EVENT_TYPE_START,
                               .payload.start = {}};
        if (!system_manager_send_joint_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    if (!system_manager_get_rtc_timestamp(manager,
                                          &manager->current_timestamp)) {
        return ATLAS_ERR_FAIL;
    }

    if (manager->is_packet_running) {
        packet_event_t event = {
            .type = PACKET_EVENT_TYPE_JOINT_READY,
            .payload.joint_ready = {.num = manager->config.joint_num,
                                    .timestamp = manager->current_timestamp,
                                    .ready = joint_ready->ready}};
        if (!system_manager_send_packet_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    } else {
        manager->is_joint_ready_pending = true;
    }

    manager->is_joint_ready = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_started_handler(
    system_manager_t* manager,
    system_event_payload_joint_started_t const* joint_started)
{
    ATLAS_ASSERT(manager && joint_started);
    ATLAS_LOG_FUNC(TAG);

    if (!system_manager_get_rtc_timestamp(manager,
                                          &manager->current_timestamp)) {
        return ATLAS_ERR_FAIL;
    }

    packet_event_t event = {
        .type = PACKET_EVENT_TYPE_JOINT_STARTED,
        .payload.joint_started = {.num = manager->config.joint_num,
                                  .timestamp = manager->current_timestamp,
                                  .started = joint_started->started}};
    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    if (manager->is_joint_start_pending) {
        manager->is_joint_start_pending = false;
    }

    manager->is_joint_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_stopped_handler(
    system_manager_t* manager,
    system_event_payload_joint_stopped_t const* joint_stopped)
{
    ATLAS_ASSERT(manager && joint_stopped);
    ATLAS_LOG_FUNC(TAG);

    if (!system_manager_get_rtc_timestamp(manager,
                                          &manager->current_timestamp)) {
        return ATLAS_ERR_FAIL;
    }

    packet_event_t event = {
        .type = PACKET_EVENT_TYPE_JOINT_STOPPED,
        .payload.joint_stopped = {.num = manager->config.joint_num,
                                  .timestamp = manager->current_timestamp,
                                  .stopped = joint_stopped->stopped}};
    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_joint_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_fault_handler(
    system_manager_t* manager,
    system_event_payload_joint_fault_t const* joint_fault)
{
    ATLAS_ASSERT(manager && joint_fault);
    ATLAS_LOG_FUNC(TAG);

    if (!system_manager_get_rtc_timestamp(manager,
                                          &manager->current_timestamp)) {
        return ATLAS_ERR_FAIL;
    }

    packet_event_t event = {
        .type = PACKET_EVENT_TYPE_JOINT_FAULT,
        .payload.joint_fault = {.num = manager->config.joint_num,
                                .timestamp = manager->current_timestamp,
                                .fault = joint_fault->fault}};
    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_measure_handler(
    system_manager_t* manager,
    system_event_payload_joint_measure_t const* joint_measure)
{
    ATLAS_ASSERT(manager && joint_measure);
    ATLAS_LOG_FUNC(TAG);

    if (atlas_joint_measure_is_equal(&manager->joint_measure,
                                     &joint_measure->measure)) {
        return ATLAS_ERR_OK;
    }

    if (!system_manager_get_rtc_timestamp(manager,
                                          &manager->current_timestamp)) {
        return ATLAS_ERR_FAIL;
    }

    packet_event_t event = {
        .type = PACKET_EVENT_TYPE_JOINT_MEASURE,
        .payload.joint_measure = {.num = manager->config.joint_num,
                                  .timestamp = manager->current_timestamp,
                                  .measure = joint_measure->measure}};
    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->joint_measure = joint_measure->measure;

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
        case SYSTEM_EVENT_TYPE_JOINT_RESET: {
            return system_manager_event_joint_reset_handler(
                manager,
                &event->payload.joint_reset);
        }
        case SYSTEM_EVENT_TYPE_JOINT_REFERENCE: {
            return system_manager_event_joint_reference_handler(
                manager,
                &event->payload.joint_reference);
        }
        case SYSTEM_EVENT_TYPE_JOINT_PARAMETERS: {
            return system_manager_event_joint_parameters_handler(
                manager,
                &event->payload.joint_parameters);
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
        case SYSTEM_EVENT_TYPE_JOINT_FAULT: {
            return system_manager_event_joint_fault_handler(
                manager,
                &event->payload.joint_fault);
        }
        case SYSTEM_EVENT_TYPE_JOINT_MEASURE: {
            return system_manager_event_joint_measure_handler(
                manager,
                &event->payload.joint_measure);
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
    manager->is_joint_ready = false;
    manager->is_joint_start_pending = false;

    memcpy(&manager->config, config, sizeof(*config));
    memset(&manager->startup_timestamp, 0, sizeof(manager->startup_timestamp));
    memset(&manager->joint_measure, 0, sizeof(manager->joint_measure));
    memset(&manager->joint_reference, 0, sizeof(manager->joint_reference));

#ifdef USE_LOG_TASK
    if (!system_manager_send_log_notify(LOG_NOTIFY_START)) {
        return ATLAS_ERR_FAIL;
    }
#endif

    if (!system_manager_get_rtc_timestamp(manager,
                                          &manager->startup_timestamp)) {
        return ATLAS_ERR_FAIL;
    }

    atlas_timestamp_print(&manager->startup_timestamp);

    return ATLAS_ERR_OK;
}
