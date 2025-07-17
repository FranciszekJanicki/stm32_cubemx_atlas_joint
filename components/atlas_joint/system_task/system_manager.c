#include "system_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

static char const* const TAG = "system_manager";

static inline bool system_manager_has_system_event()
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_SYSTEM));
}

static inline bool system_manager_send_joints_event(joints_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_JOINTS), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_send_packet_event(packet_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_PACKET), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_send_safety_event(safety_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_SAFETY), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_receive_system_event(system_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_SYSTEM), event, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_receive_system_notify(system_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0, SYSTEM_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_start_retry_timer(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    xTimerStart(timer_manager_get(TIMER_TYPE_SYSTEM), pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool system_manager_stop_retry_timer(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return xTimerStop(timer_manager_get(TIMER_TYPE_SYSTEM), pdMS_TO_TICKS(1)) == pdPASS;
}

static atlas_err_t system_manager_notify_joints_ready_handler(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    manager->is_joints_ready = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_packet_ready_handler(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    packet_event_t event = {.type = PACKET_EVENT_TYPE_START};

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_retry_timer_handler(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    bool should_retry = true;

    if (manager->is_joints_ready) {
        joints_event_t event = {.type = JOINTS_EVENT_TYPE_START};

        if (!system_manager_send_joints_event(&event)) {
            return ATLAS_ERR_FAIL;
        }

        should_retry = false;
    }

    if (manager->is_safety_ready) {
        safety_event_t event = {.type = SAFETY_EVENT_TYPE_START};

        if (!system_manager_send_safety_event(&event)) {
            return ATLAS_ERR_FAIL;
        }

        should_retry = false;
    }

    if (should_retry) {
        if (!system_manager_start_retry_timer(manager)) {
            return ATLAS_ERR_FAIL;
        }
    } else {
        if (!system_manager_stop_retry_timer(manager)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_notify_safety_ready_handler(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    safety_event_t event = {.type = SAFETY_EVENT_TYPE_START};

    if (!system_manager_send_safety_event(&event)) {
        return ATLAS_ERR_FAIL;
    }
}

static atlas_err_t system_manager_notify_handler(system_manager_t* manager, system_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & SYSTEM_NOTIFY_JOINTS_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_joints_ready_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_PACKET_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_packet_ready_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_RETRY_TIMER) {
        ATLAS_RET_ON_ERR(system_manager_notify_retry_timer_handler(manager));
    }
    if (notify & SYSTEM_NOTIFY_SAFETY_READY) {
        ATLAS_RET_ON_ERR(system_manager_notify_safety_ready_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_measured_joints_data_handler(
    system_manager_t* manager,
    system_event_payload_joints_data_t const* joints_data)
{
    ATLAS_ASSERT(manager && joints_data);
    ATLAS_LOG_FUNC(TAG);

    packet_event_t event = {.type = PACKET_EVENT_TYPE_JOINTS_DATA};
    event.payload.joints_data = *joints_data;

    if (!system_manager_send_packet_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_reference_joints_data_handler(
    system_manager_t* manager,
    system_event_payload_joints_data_t const* joints_data)
{
    ATLAS_ASSERT(manager && joints_data);
    ATLAS_LOG_FUNC(TAG);

    joints_event_t event = {.type = JOINTS_EVENT_TYPE_REFERENCE_DATA};
    event.payload.reference_data = *joints_data;

    if (!system_manager_send_joints_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_start_joints_handler(
    system_manager_t* manager,
    system_event_payload_start_joints_t const* start_joints)
{
    ATLAS_ASSERT(manager && start_joints);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_joints_ready) {
        joints_event_t event = {.type = JOINTS_EVENT_TYPE_START};
        event.payload.start = *start_joints;

        if (!system_manager_send_joints_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    } else {
        if (!system_manager_start_retry_timer(manager)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_stop_joints_handler(
    system_manager_t* manager,
    system_event_payload_stop_joints_t const* stop_joints)
{
    ATLAS_ASSERT(manager && stop_joints);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_joints_ready) {
        joints_event_t event = {.type = JOINTS_EVENT_TYPE_STOP};
        event.payload.stop = *stop_joints;

        if (!system_manager_send_joints_event(&event)) {
            return ATLAS_ERR_FAIL;
        }
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_safety_trig_handler(
    system_manager_t* manager,
    system_event_payload_safety_trig_t const* safety_trig)
{
    ATLAS_ASSERT(manager && safety_trig);
    ATLAS_LOG_FUNC(TAG);

    joints_event_t event = {.type = JOINTS_EVENT_TYPE_STOP};

    if (!system_manager_send_joints_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t system_manager_event_handler(system_manager_t* manager,
                                                system_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case SYSTEM_EVENT_TYPE_JOINTS_DATA: {
            switch (event->origin) {
                case SYSTEM_EVENT_ORIGIN_JOINTS: {
                    return system_manager_event_measured_joints_data_handler(
                        manager,
                        &event->payload.joints_data);
                }
                case SYSTEM_EVENT_ORIGIN_PACKET: {
                    system_manager_event_reference_joints_data_handler(manager,
                                                                       &event->payload.joints_data);
                }
                default: {
                    return ATLAS_ERR_UNKNOWN_ORIGIN;
                }
            }
        }
        case SYSTEM_EVENT_TYPE_START_JOINTS: {
            return system_manager_event_start_joints_handler(manager, &event->payload.start_joints);
        }
        case SYSTEM_EVENT_TYPE_STOP_JOINTS: {
            return system_manager_event_stop_joints_handler(manager, &event->payload.stop_joints);
        }
        case SYSTEM_EVENT_TYPE_SAFETY_TRIG: {
            return system_manager_event_safety_trig_handler(manager, &event->payload.safety_trig);
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

atlas_err_t system_manager_initialize(system_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    manager->is_running = true;
    manager->is_joints_ready = false;
    manager->is_packet_ready = false;
    manager->is_safety_ready = false;

    return ATLAS_ERR_OK;
}
