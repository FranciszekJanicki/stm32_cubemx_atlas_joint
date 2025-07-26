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

static inline bool system_manager_send_joint_event(system_manager_t* manager,
                                                   joint_event_t const* event)
{
    ATLAS_ASSERT(event);

    if (!manager->is_joint_running && event->type != JOINT_EVENT_TYPE_START) {
        return false;
    }

    return xQueueSend(queue_manager_get(QUEUE_TYPE_JOINT),
                      event,
                      pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_send_packet_event(system_manager_t* manager,
                                                    packet_event_t const* event)
{
    ATLAS_ASSERT(event);

    if (!manager->is_packet_running && event->type != PACKET_EVENT_TYPE_START) {
        return false;
    }

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

static inline bool system_manager_send_uart_notify(uart_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_UART), notify, eSetBits) ==
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

static inline bool system_manager_start_retry_timer(void)
{
    return xTimerStart(timer_manager_get(TIMER_TYPE_SYSTEM),
                       pdMS_TO_TICKS(1)) == pdPASS;
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

    bool (*retry_timer_function)(void) = NULL;

    if (manager->is_packet_running) {
        packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINT_READY};
        event.payload.joint_ready = (packet_event_payload_joint_ready_t){};

        if (!system_manager_send_packet_event(manager, &event)) {
            return ATLAS_ERR_FAIL;
        }

        retry_timer_function = system_manager_stop_retry_timer;
    } else {
        retry_timer_function = system_manager_start_retry_timer;
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

    if (!system_manager_send_packet_event(manager, &event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_packet_running = true;

#ifdef PACKET_TEST
    HAL_TIM_Base_Start_IT(manager->config.packet_ready_timer);
#endif

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

        if (!system_manager_send_packet_event(manager, &event)) {
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

    if (!system_manager_send_packet_event(manager, &event)) {
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

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_START};
    event.payload.start = (joint_event_payload_start_t){};

    if (!system_manager_send_joint_event(manager, &event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_joint_running = true;

    system_manager_get_rtc_timestamp(manager, &manager->start_timestamp);
    atlas_timestamp_print(&manager->start_timestamp);

#ifdef PACKET_TEST
    HAL_TIM_Base_Start_IT(manager->config.delta_timer);
#endif 

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_stop_handler(
    system_manager_t* manager,
    system_event_payload_joint_stop_t const* joint_stop)
{
    ATLAS_ASSERT(manager && joint_stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_STOP};
    event.payload.stop = (joint_event_payload_stop_t){};

    if (!system_manager_send_joint_event(manager, &event)) {
        return ATLAS_ERR_FAIL;
    }

    memset(&manager->start_timestamp, 0, sizeof(manager->start_timestamp));

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_reference_handler(
    system_manager_t* manager,
    system_event_payload_joint_reference_t const* joint_reference)
{
    ATLAS_ASSERT(manager && joint_reference);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (atlas_joint_reference_is_equal(&manager->joint_reference,
                                       joint_reference)) {
        return ATLAS_ERR_OK;
    }

    joint_event_t event = {.type = JOINT_EVENT_TYPE_REFERENCE};
    event.payload.reference = *joint_reference;

    if (!system_manager_send_joint_event(manager, &event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->joint_reference = *joint_reference;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_joint_measure_handler(
    system_manager_t* manager,
    system_event_payload_joint_measure_t const* joint_measure)
{
    ATLAS_ASSERT(manager && joint_measure);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (atlas_joint_measure_is_equal(&manager->joint_measure, joint_measure)) {
        return ATLAS_ERR_OK;
    }

    packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINT_MEASURE};
    event.payload.joint_measure.num = manager->config.num;
    event.payload.joint_measure.measure = *joint_measure;
    event.payload.joint_measure.timestamp = manager->current_timestamp;

    if (!system_manager_send_packet_event(manager, &event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->joint_measure = *joint_measure;

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
        case SYSTEM_EVENT_TYPE_JOINT_REFERENCE: {
            return system_manager_event_joint_reference_handler(
                manager,
                &event->payload.joint_reference);
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

    manager->is_running = true;
    manager->is_packet_running = false;
    manager->is_joint_running = false;
    manager->config = *config;

    if (!system_manager_send_uart_notify(UART_NOTIFY_START)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}
