#include "packet_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "task.h"
#include <stdint.h>

static char const* const TAG = "packet_manager";

static inline bool packet_manager_has_packet_event()
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_PACKET));
}

static inline bool packet_manager_send_system_event(system_event_t const* event)
{
    ATLAS_ASSERT(event);

    return xQueueSend(queue_manager_get(QUEUE_TYPE_SYSTEM),
                      event,
                      pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool packet_manager_send_system_notify(system_notify_t notify)
{
    return xTaskNotify(task_manager_get(TASK_TYPE_SYSTEM), notify, eSetBits) ==
           pdPASS;
}

static inline bool packet_manager_receive_packet_event(packet_event_t* event)
{
    ATLAS_ASSERT(event);

    return xQueueReceive(queue_manager_get(QUEUE_TYPE_PACKET),
                         event,
                         pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool packet_manager_receive_packet_notify(packet_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0,
                           PACKET_NOTIFY_ALL,
                           (uint32_t*)notify,
                           pdMS_TO_TICKS(1)) == pdPASS;
}

static inline bool packet_manager_send_robot_packet(
    packet_manager_t* manager,
    atlas_robot_packet_t const* packet)
{
    ATLAS_ASSERT(manager && packet);

    return true;
    // HAL_SPI_Transmit(manager->config.packet_spi,
    // (uint8_t*)packet, sizeof(*packet), 100) == HAL_OK;
}

static inline bool packet_manager_receive_joint_packet(
    packet_manager_t* manager,
    atlas_joint_packet_t* packet)
{
    ATLAS_ASSERT(manager && packet);

    return true;
    // HAL_SPI_Receive(manager->config.packet_spi,
    // (uint8_t*)packet, sizeof(*packet), 100) == HAL_OK;
}

static inline void packet_manager_set_robot_packet_ready_pin(
    packet_manager_t* manager,
    bool state)
{
    ATLAS_ASSERT(manager);

    HAL_GPIO_WritePin(manager->config.robot_packet_ready_gpio,
                      manager->config.robot_packet_ready_pin,
                      (GPIO_PinState)state);
}

static atlas_err_t packet_manager_packet_joint_start_handler(
    packet_manager_t* manager,
    atlas_joint_packet_payload_joint_start_t const* joint_start)
{
    ATLAS_ASSERT(manager && joint_start);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_JOINT_START};
    event.payload.joint_start = *joint_start;

    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_packet_joint_stop_handler(
    packet_manager_t* manager,
    atlas_joint_packet_payload_joint_stop_t const* joint_stop)
{
    ATLAS_ASSERT(manager && joint_stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_JOINT_STOP};
    event.payload.joint_stop = *joint_stop;

    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_packet_joint_data_handler(
    packet_manager_t* manager,
    atlas_joint_packet_payload_joint_data_t const* joint_data)
{
    ATLAS_ASSERT(manager && joint_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_JOINT_DATA};
    event.payload.joint_data.position = joint_data->position;

    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_joint_packet_handler(
    packet_manager_t* manager,
    atlas_joint_packet_t const* packet)
{
    ATLAS_ASSERT(manager && packet);
    ATLAS_LOG_FUNC(TAG);

    switch (packet->type) {
        case ATLAS_JOINT_PACKET_TYPE_JOINT_START: {
            return packet_manager_packet_joint_start_handler(
                manager,
                &packet->payload.joint_start);
        }
        case ATLAS_JOINT_PACKET_TYPE_JOINT_STOP: {
            return packet_manager_packet_joint_stop_handler(
                manager,
                &packet->payload.joint_stop);
        }
        case ATLAS_JOINT_PACKET_TYPE_JOINT_DATA: {
            return packet_manager_packet_joint_data_handler(
                manager,
                &packet->payload.joint_data);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_PACKET;
        }
    }
}

static atlas_err_t packet_manager_notify_joint_packet_ready_handler(
    packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_joint_packet_t packet;
    if (packet_manager_receive_joint_packet(manager, &packet)) {
        ATLAS_RET_ON_ERR(packet_manager_joint_packet_handler(manager, &packet));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_notify_handler(packet_manager_t* manager,
                                                 packet_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if (notify & PACKET_NOTIFY_JOINT_PACKET_READY) {
        ATLAS_RET_ON_ERR(
            packet_manager_notify_joint_packet_ready_handler(manager));
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_start_handler(
    packet_manager_t* manager,
    packet_event_payload_start_t const* start)
{
    ATLAS_ASSERT(manager && start);
    ATLAS_LOG_FUNC(TAG);

    if (manager->is_running) {
        return ATLAS_ERR_ALREADY_RUNNING;
    }

    packet_manager_set_robot_packet_ready_pin(manager, true);

    manager->is_running = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_stop_handler(
    packet_manager_t* manager,
    packet_event_payload_stop_t const* stop)
{
    ATLAS_ASSERT(manager && stop);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    packet_manager_set_robot_packet_ready_pin(manager, true);

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_joint_data_handler(
    packet_manager_t* manager,
    packet_event_payload_joint_data_t const* joint_data)
{
    ATLAS_ASSERT(manager && joint_data);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_robot_packet_t packet = {.type = ATLAS_ROBOT_PACKET_TYPE_JOINT_DATA};
    packet.payload.joint_data.position = joint_data->position;

    ATLAS_RET_ON_ERR(packet_manager_send_robot_packet(manager, &packet));

    packet_manager_set_robot_packet_ready_pin(manager, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    packet_manager_set_robot_packet_ready_pin(manager, true);

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_joint_fault_handler(
    packet_manager_t* manager,
    packet_event_payload_joint_fault_t const* joint_fault)
{
    ATLAS_ASSERT(manager && joint_fault);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_robot_packet_t packet = {.type = ATLAS_ROBOT_PACKET_TYPE_JOINT_FAULT};
    packet.payload.joint_fault = *joint_fault;

    ATLAS_RET_ON_ERR(packet_manager_send_robot_packet(manager, &packet));

    packet_manager_set_robot_packet_ready_pin(manager, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    packet_manager_set_robot_packet_ready_pin(manager, true);

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_joint_ready_handler(
    packet_manager_t* manager,
    packet_event_payload_joint_ready_t const* joint_ready)
{
    ATLAS_ASSERT(manager && joint_ready);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_robot_packet_t packet = {.type = ATLAS_ROBOT_PACKET_TYPE_JOINT_READY};
    packet.payload.joint_ready = *joint_ready;

    ATLAS_RET_ON_ERR(packet_manager_send_robot_packet(manager, &packet));

    packet_manager_set_robot_packet_ready_pin(manager, false);
    vTaskDelay(pdMS_TO_TICKS(10));
    packet_manager_set_robot_packet_ready_pin(manager, true);

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_handler(packet_manager_t* manager,
                                                packet_event_t const* event)
{
    ATLAS_ASSERT(manager && event);

    switch (event->type) {
        case PACKET_EVENT_TYPE_START: {
            return packet_manager_event_start_handler(manager,
                                                      &event->payload.start);
        }
        case PACKET_EVENT_TYPE_STOP: {
            return packet_manager_event_stop_handler(manager,
                                                     &event->payload.stop);
        }
        case PACKET_EVENT_TYPE_JOINT_DATA: {
            return packet_manager_event_joint_data_handler(
                manager,
                &event->payload.joint_data);
        }
        case PACKET_EVENT_TYPE_JOINT_FAULT: {
            return packet_manager_event_joint_fault_handler(
                manager,
                &event->payload.joint_fault);
        }
        case PACKET_EVENT_TYPE_JOINT_READY: {
            return packet_manager_event_joint_ready_handler(
                manager,
                &event->payload.joint_ready);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_EVENT;
        }
    }
}

atlas_err_t packet_manager_process(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    packet_notify_t notify;
    if (packet_manager_receive_packet_notify(&notify)) {
        ATLAS_RET_ON_ERR(packet_manager_notify_handler(manager, notify));
    }

    packet_event_t event;
    while (packet_manager_has_packet_event()) {
        if (packet_manager_receive_packet_event(&event)) {
            ATLAS_RET_ON_ERR(packet_manager_event_handler(manager, &event));
        }
    }

    return ATLAS_ERR_OK;
}

atlas_err_t packet_manager_initialize(packet_manager_t* manager,
                                      packet_config_t const* config)
{
    ATLAS_ASSERT(manager && config);

    manager->is_running = false;
    manager->config = *config;

    packet_manager_set_robot_packet_ready_pin(manager, true);

    if (!packet_manager_send_system_notify(SYSTEM_NOTIFY_PACKET_READY)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

void packet_ready_callback(void)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_PACKET),
                       PACKET_NOTIFY_JOINT_PACKET_READY,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}