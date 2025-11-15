#include "packet_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

static char const* const TAG = "atlas_joint:packet_manager";

static inline bool packet_manager_has_packet_event(void)
{
    return uxQueueMessagesWaiting(queue_manager_get(QUEUE_TYPE_PACKET)) > 0U;
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

static inline bool packet_manager_set_rtc_timestamp(
    packet_manager_t* manager,
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

static inline bool packet_manager_get_rtc_timestamp(
    packet_manager_t* manager,
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

static inline bool packet_manager_update_startup_timestamp(
    packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return packet_manager_get_rtc_timestamp(manager,
                                            &manager->startup_timestamp);
}

static inline bool packet_manager_update_current_timestamp(
    packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return packet_manager_get_rtc_timestamp(manager,
                                            &manager->current_timestamp);
}

static inline bool packet_manager_packet_spi_transfer(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    size_t transfer_size =
        sizeof(manager->transmit_buffer) > sizeof(manager->receive_buffer)
            ? sizeof(manager->transmit_buffer)
            : sizeof(manager->receive_buffer);

    return HAL_SPI_TransmitReceive_IT(manager->config.packet_spi_bus,
                                      manager->transmit_buffer,
                                      manager->receive_buffer,
                                      transfer_size) == HAL_OK;
}

static inline bool packet_manager_packet_spi_transmit(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_SPI_Transmit_IT(manager->config.packet_spi_bus,
                               manager->transmit_buffer,
                               sizeof(manager->transmit_buffer)) == HAL_OK;
}

static inline bool packet_manager_packet_spi_receive(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_SPI_Receive_IT(manager->config.packet_spi_bus,
                              manager->receive_buffer,
                              sizeof(manager->receive_buffer)) == HAL_OK;
}

static inline void packet_manager_set_data_ready_pin(packet_manager_t* manager,
                                                     bool state)
{
    ATLAS_ASSERT(manager);

    HAL_GPIO_WritePin(manager->config.data_ready_gpio,
                      manager->config.data_ready_pin,
                      (GPIO_PinState)state);
}

static inline void packet_manager_assert_data_ready(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    packet_manager_set_data_ready_pin(manager, false);

    manager->is_transfer_pending = true;
    manager->is_transfer_complete = false;
}

static inline void packet_manager_deassert_data_ready(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    packet_manager_set_data_ready_pin(manager, true);

    manager->is_transfer_pending = false;
    manager->is_transfer_complete = false;
}

static inline void packet_manager_prepare_robot_packet(
    packet_manager_t* manager,
    atlas_robot_packet_t const* packet)
{
    ATLAS_ASSERT(manager && packet);

    memset(manager->transmit_buffer, 0, sizeof(manager->transmit_buffer));

    atlas_robot_packet_print(packet);
    atlas_robot_packet_encode_binary(
        packet,
        (uint8_t (*)[ATLAS_ROBOT_PACKET_SIZE])manager->transmit_buffer);

    packet_manager_assert_data_ready(manager);

    manager->is_transfer_pending = true;
    manager->is_transfer_complete = false;
}

static inline bool packet_manager_send_robot_packet(packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    if (!packet_manager_packet_spi_transfer(manager)) {
        return false;
    }

    manager->is_transfer_pending = false;

    return true;
}

static inline void packet_manager_parse_joint_packet(
    packet_manager_t* manager,
    atlas_joint_packet_t* packet)
{
    ATLAS_ASSERT(manager && packet);

    atlas_joint_packet_decode_binary(
        (uint8_t (*)[ATLAS_JOINT_PACKET_SIZE])manager->receive_buffer,
        packet);
    atlas_joint_packet_print(packet);
}

static inline bool packet_manager_receive_joint_packet(
    packet_manager_t* manager,
    atlas_joint_packet_t* packet)
{
    ATLAS_ASSERT(manager && packet);

    return packet_manager_packet_spi_transfer(manager);
}

static atlas_err_t packet_manager_packet_joint_command_handler(
    packet_manager_t* manager,
    atlas_joint_packet_payload_joint_command_t const* joint_command)
{
    ATLAS_ASSERT(manager && joint_command);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_JOINT_COMMAND,
                            .payload.joint_command.command = *joint_command};
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

    if (packet->destination != manager->config.joint_num) {
        ATLAS_ERR_FAIL;
    }

    switch (packet->type) {
        case ATLAS_JOINT_PACKET_TYPE_JOINT_COMMAND: {
            return packet_manager_packet_joint_command_handler(
                manager,
                &packet->payload.joint_command);
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

#ifdef JOINT_PACKET_TEST
    static int i = 0;

    atlas_joint_command_t command = {.type = ATLAS_JOINT_COMMAND_TYPE_SET_STATE,
                                     .payload.set_state.state =
                                         ATLAS_JOINT_STATE_RUNNING};

    system_event_t event = {.type = SYSTEM_EVENT_TYPE_JOINT_COMMAND,
                            .origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .payload.joint_command.command = command};
    packet_manager_send_system_event(&event);
#endif

#ifdef ROBOT_PACKET_TEST
    atlas_joint_response_t response = {
        .type = ATLAS_JOINT_RESPONSE_TYPE_GET_STATE,
        .payload.get_state = {.success = true,
                              .state = ATLAS_JOINT_STATE_RUNNING}};

    atlas_robot_packet_t packet = {.type =
                                       ATLAS_ROBOT_PACKET_TYPE_JOINT_RESPONSE,
                                   .origin = manager->config.joint_num,
                                   .timestamp = manager->current_timestamp,
                                   .payload.joint_response = response};
    packet_manager_prepare_robot_packet(manager, &packet);
#endif

#if !defined(JOINT_PACKET_TEST) && !defined(ROBOT_PACKET_TEST)
    if (manager->is_transfer_pending && !manager->is_transfer_complete) {
        if (!packet_manager_send_robot_packet(manager)) {
            return ATLAS_ERR_FAIL;
        }
    } else if (!manager->is_transfer_pending && manager->is_transfer_complete) {
        packet_manager_deassert_data_ready(manager);
    }

    atlas_joint_packet_t packet;
    if (packet_manager_receive_joint_packet(manager, &packet)) {
        ATLAS_RET_ON_ERR(packet_manager_joint_packet_handler(manager, &packet));
    }
#endif

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_notify_transfer_complete_handler(
    packet_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    manager->is_transfer_complete = true;

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

    if ((notify & PACKET_NOTIFY_SLAVE_SELECT) == PACKET_NOTIFY_SLAVE_SELECT) {
        ATLAS_RET_ON_ERR(packet_manager_notify_slave_select_handler(manager));
    }
    if ((notify & PACKET_NOTIFY_TRANSFER_COMPLETE) ==
        PACKET_NOTIFY_TRANSFER_COMPLETE) {
        ATLAS_RET_ON_ERR(
            packet_manager_notify_transfer_complete_handler(manager));
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

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_PACKET_STARTED,
                            .payload.packet_started = {}};
    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_running = true;

#ifdef JOINT_PACKET_TEST
    xTimerStart(timer_manager_get(TIMER_TYPE_JOINT_PACKET_TEST),
                pdMS_TO_TICKS(1));
#endif

#ifdef ROBOT_PACKET_TEST
    xTimerStart(timer_manager_get(TIMER_TYPE_ROBOT_PACKET_TEST),
                pdMS_TO_TICKS(1));
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

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_PACKET_STOPPED,
                            .payload.packet_stopped = {}};
    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_running = false;

#ifdef JOINT_PACKET_TEST
    xTimerStop(timer_manager_get(TIMER_TYPE_JOINT_PACKET_TEST),
               pdMS_TO_TICKS(1));
#endif

#ifdef ROBOT_PACKET_TEST
    xTimerStop(timer_manager_get(TIMER_TYPE_ROBOT_PACKET_TEST),
               pdMS_TO_TICKS(1));
#endif

    return ATLAS_ERR_OK;
}

static atlas_err_t packet_manager_event_joint_response_handler(
    packet_manager_t* manager,
    packet_event_payload_joint_response_t const* joint_response)
{
    ATLAS_ASSERT(manager && joint_response);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_running) {
        return ATLAS_ERR_NOT_RUNNING;
    }

    if (!packet_manager_update_current_timestamp(manager)) {
        return ATLAS_ERR_FAIL;
    }

    atlas_robot_packet_t packet = {
        .type = ATLAS_ROBOT_PACKET_TYPE_JOINT_RESPONSE,
        .origin = manager->config.joint_num,
        .timestamp = manager->current_timestamp,
        .payload.joint_response = joint_response->response};

    packet_manager_prepare_robot_packet(manager, &packet);

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
        case PACKET_EVENT_TYPE_JOINT_RESPONSE: {
            return packet_manager_event_joint_response_handler(
                manager,
                &event->payload.joint_response);
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

    manager->config = *config;
    manager->is_running = false;
    manager->is_transfer_complete = false;
    manager->is_transfer_pending = false;

    memset(&manager->startup_timestamp, 0, sizeof(manager->startup_timestamp));
    memset(&manager->current_timestamp, 0, sizeof(manager->current_timestamp));

    memset(manager->receive_buffer, 0, sizeof(manager->receive_buffer));
    memset(manager->transmit_buffer, 0, sizeof(manager->transmit_buffer));

    if (packet_manager_packet_spi_transfer(manager)) {
        return ATLAS_ERR_FAIL;
    }
    packet_manager_deassert_data_ready(manager);

    if (!packet_manager_update_startup_timestamp(manager) ||
        !packet_manager_update_current_timestamp(manager)) {
        return ATLAS_ERR_FAIL;
    }

    system_event_t event = {.origin = SYSTEM_EVENT_ORIGIN_PACKET,
                            .type = SYSTEM_EVENT_TYPE_PACKET_READY,
                            .payload.packet_ready = {}};
    if (!packet_manager_send_system_event(&event)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}
