#include "packet_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

static char const* const TAG = "packet_manager";

static inline bool packet_manager_has_packet_event(void)
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

static inline bool packet_manager_packet_spi_transmit_buffer(
    packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    memset(manager->receive_buffer, 0, sizeof(manager->receive_buffer));

    return HAL_SPI_TransmitReceive(manager->config.packet_spi_bus,
                                   manager->transmit_buffer,
                                   manager->receive_buffer,
                                   sizeof(manager->transmit_buffer),
                                   HAL_MAX_DELAY) == HAL_OK;
}

static inline bool packet_manager_packet_spi_receive_buffer(
    packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    memset(manager->transmit_buffer, 0, sizeof(manager->transmit_buffer));

    return HAL_SPI_TransmitReceive(manager->config.packet_spi_bus,
                                   manager->transmit_buffer,
                                   manager->receive_buffer,
                                   sizeof(manager->receive_buffer),
                                   HAL_MAX_DELAY) == HAL_OK;
}

static inline void packet_manager_set_data_ready_pin(packet_manager_t* manager,
                                                     bool state)
{
    ATLAS_ASSERT(manager);

    HAL_GPIO_WritePin(manager->config.data_ready_gpio,
                      manager->config.data_ready_pin,
                      (GPIO_PinState)state);
}

static inline bool packet_manager_prepare_robot_packet(
    packet_manager_t* manager,
    atlas_robot_packet_t const* packet)
{
    ATLAS_ASSERT(manager && packet);

    memset(manager->transmit_buffer, 0, sizeof(manager->transmit_buffer));

    atlas_robot_packet_print(packet);
    atlas_robot_packet_encode(packet, &manager->transmit_buffer);

    packet_manager_set_data_ready_pin(manager, false);

    manager->is_waiting_for_transmit = true;
    manager->is_transmit_sent = false;
    manager->is_transmit_finished = false;

    return true;
}

static inline bool packet_manager_send_robot_packet(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    if (!packet_manager_packet_spi_transmit_buffer(manager)) {
        return false;
    }

    manager->is_transmit_sent = true;

    return true;
}

static inline bool packet_manager_receive_joint_packet(
    packet_manager_t* manager,
    atlas_joint_packet_t* packet)
{
    ATLAS_ASSERT(manager && packet);

    if (!packet_manager_packet_spi_receive_buffer(manager)) {
        return false;
    }

    atlas_joint_packet_decode(&manager->receive_buffer, packet);
    atlas_joint_packet_print(packet);

    return true;
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

static atlas_err_t packet_manager_packet_joint_reference_handler(
    packet_manager_t* manager,
    atlas_joint_packet_payload_joint_reference_t const* joint_reference)
{
    ATLAS_ASSERT(manager && joint_reference);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_JOINT_REFERENCE};
    event.payload.joint_reference = *joint_reference;

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
        case ATLAS_JOINT_PACKET_TYPE_JOINT_REFERENCE: {
            return packet_manager_packet_joint_reference_handler(
                manager,
                &packet->payload.joint_reference);
        }
        default: {
            return ATLAS_ERR_UNKNOWN_PACKET;
        }
    }
}

static atlas_err_t packet_manager_notify_slave_select_handler(
    packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

#ifdef PACKET_TEST
    system_event_t event = {.type = SYSTEM_EVENT_TYPE_JOINT_REFERENCE,
                            .origin = SYSTEM_EVENT_ORIGIN_PACKET};

    event.payload.joint_reference.position = 300.0F;
    event.payload.joint_reference.delta_time = 0.001F;
    packet_manager_send_system_event(&event);
#else

    if (manager->is_waiting_for_transmit && !manager->is_transmit_sent &&
        !manager->is_transmit_finished) {
        if (!packet_manager_send_robot_packet(manager)) {
            return false;
        }

        manager->is_transmit_sent = true;
    } else if (manager->is_waiting_for_transmit && manager->is_transmit_sent &&
               !manager->is_transmit_finished) {
        packet_manager_set_data_ready_pin(manager, true);

        manager->is_transmit_finished = true;
        manager->is_waiting_for_transmit = false;
    } else if (!manager->is_waiting_for_transmit) {
        atlas_joint_packet_t packet;
        if (packet_manager_receive_joint_packet(manager, &packet)) {
            ATLAS_RET_ON_ERR(
                packet_manager_joint_packet_handler(manager, &packet));
        }
    }
#endif

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_notify_handler(packet_manager_t* manager,
                                                 packet_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if ((notify & PACKET_NOTIFY_SLAVE_SELECT) == PACKET_NOTIFY_SLAVE_SELECT) {
        ATLAS_RET_ON_ERR(packet_manager_notify_slave_select_handler(manager));
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

    manager->is_running = true;

#ifdef PACKET_TEST
    xTimerStart(timer_manager_get(TIMER_TYPE_PACKET_TEST), pdMS_TO_TICKS(1));
#endif

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

    manager->is_running = false;

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_joint_measure_handler(
    packet_manager_t* manager,
    packet_event_payload_joint_measure_t const* joint_measure)
{
    ATLAS_ASSERT(manager && joint_measure);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    atlas_robot_packet_t packet = {.type =
                                       ATLAS_ROBOT_PACKET_TYPE_JOINT_MEASURE};
    packet.origin = joint_measure->num;
    packet.timestamp = joint_measure->timestamp;
    packet.payload.joint_measure = joint_measure->measure;

#ifndef PACKET_TEST
    if (!packet_manager_prepare_robot_packet(manager, &packet)) {
        return ATLAS_ERR_FAIL;
    }
#endif

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
    packet.origin = joint_fault->num;
    packet.timestamp = joint_fault->timestamp;
    packet.payload.joint_fault = joint_fault->fault;

    if (!packet_manager_prepare_robot_packet(manager, &packet)) {
        return ATLAS_ERR_FAIL;
    }

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
    packet.origin = joint_ready->num;
    packet.timestamp = joint_ready->timestamp;
    packet.payload.joint_ready = joint_ready->ready;

#ifdef PACKET_TEST
    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_JOINT_START};
    packet_manager_send_system_event(&event);
#else
    if (!packet_manager_prepare_robot_packet(manager, &packet)) {
        return ATLAS_ERR_FAIL;
    }
#endif

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
        case PACKET_EVENT_TYPE_JOINT_MEASURE: {
            return packet_manager_event_joint_measure_handler(
                manager,
                &event->payload.joint_measure);
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
        ATLAS_LOG_ON_ERR(TAG, packet_manager_notify_handler(manager, notify));
    }

    packet_event_t event;
    while (packet_manager_has_packet_event()) {
        if (packet_manager_receive_packet_event(&event)) {
            ATLAS_LOG_ON_ERR(TAG,
                             packet_manager_event_handler(manager, &event));
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

    memset(manager->receive_buffer, 0, sizeof(manager->receive_buffer));
    memset(manager->transmit_buffer, 0, sizeof(manager->transmit_buffer));

    packet_manager_set_data_ready_pin(manager, true);

    if (!packet_manager_send_system_notify(SYSTEM_NOTIFY_PACKET_READY)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}
