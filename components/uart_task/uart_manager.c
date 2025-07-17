#include "uart_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static size_t uart_manager_receive_stream_data(StreamBufferHandle_t stream_buffer,
                                               uint8_t* stream_data,
                                               size_t stream_data_size)
{
    assert(stream_buffer && stream_data);

    return xStreamBufferReceive(stream_buffer, stream_data, stream_data_size, portMAX_DELAY);
}

static size_t uart_manager_transmit_stream_data(StreamBufferHandle_t stream_buffer,
                                                uint8_t const* stream_data,
                                                size_t stream_data_size)
{
    assert(stream_buffer && stream_data);

    return xStreamBufferSend(stream_buffer, stream_data, stream_data_size, portMAX_DELAY);
}

static uart_err_t uart_manager_receive_uart_data(UART_HandleTypeDef* uart,
                                                 uint8_t* uart_data,
                                                 size_t uart_data_size)
{
    assert(uart && uart_data);

    return HAL_UART_Receive_IT(uart, uart_data, uart_data_size) == HAL_OK ? UART_ERR_OK
                                                                          : UART_ERR_RECEIVE;
}

static uart_err_t uart_manager_transmit_uart_data(UART_HandleTypeDef* uart,
                                                  uint8_t const* uart_data,
                                                  size_t uart_data_size)
{
    assert(uart && uart_data);

    return HAL_UART_Transmit_IT(uart, uart_data, uart_data_size) == HAL_OK ? UART_ERR_OK
                                                                           : UART_ERR_TRANSMIT;
}

static bool uart_manager_receive_uart_notify(uart_notify_t* notify)
{
    assert(notify);

    return xTaskNotifyWait(0, UART_NOTIFY_ALL, (uint32_t*)notify, pdMS_TO_TICKS(10)) == pdPASS;
}

static uart_err_t uart_manager_notify_receive_complete_handler(uart_manager_t* manager)
{
    assert(manager);

    memset(manager->uart_buffer, 0, sizeof(manager->uart_buffer));

    uart_err_t err = uart_manager_receive_uart_data(manager->uart,
                                                    manager->uart_buffer,
                                                    manager->uart_buffer_size);
    if (err != UART_ERR_OK) {
        return err;
    }

    size_t transmitted_size = uart_manager_transmit_stream_data(manager->stream_buffer,
                                                                manager->uart_buffer,
                                                                manager->uart_buffer_size);
    if (transmitted_size == 0U) {
        return UART_ERR_FAIL;
    }

    return err;
}

static uart_err_t uart_manager_notify_transmit_complete_handler(uart_manager_t* manager)
{
    assert(manager);

    memset(manager->uart_buffer, 0, sizeof(manager->uart_buffer));

    size_t received_size = uart_manager_receive_stream_data(manager->stream_buffer,
                                                            manager->uart_buffer,
                                                            manager->uart_buffer_size);
    if (received_size == 0U) {
        return UART_ERR_FAIL;
    }

    return uart_manager_transmit_uart_data(manager->uart, manager->uart_buffer, received_size);
}

static uart_err_t uart_manager_notify_handler(uart_manager_t* manager, uart_notify_t notify)
{
    assert(manager);

    uart_err_t err = UART_ERR_OK;

    if (notify & UART_NOTIFY_TRANSMIT_COMPLETE) {
        err = uart_manager_notify_transmit_complete_handler(manager);
        if (err != UART_ERR_OK) {
            return err;
        }
    }
    if (notify & UART_NOTIFY_RECEIVE_COMPLETE) {
        return uart_manager_notify_receive_complete_handler(manager);
    }

    return UART_ERR_OK;
}

uart_err_t uart_manager_process(uart_manager_t* manager)
{
    assert(manager);

    uart_notify_t notify;
    if (uart_manager_receive_uart_notify(&notify)) {
        return uart_manager_notify_handler(manager, notify);
    }

    return UART_ERR_OK;
}

uart_err_t uart_manager_initialize(uart_manager_t* manager,
                                   StreamBufferHandle_t stream_buffer,
                                   UART_HandleTypeDef* uart,
                                   uint8_t* uart_buffer,
                                   size_t uart_buffer_size,
                                   uart_action_t action)
{
    manager->stream_buffer = stream_buffer;
    manager->uart = uart;
    manager->uart_buffer = uart_buffer;
    manager->uart_buffer_size = uart_buffer_size;

    switch (action) {
        case UART_ACTION_RECEIVE:
            return uart_manager_notify_receive_complete_handler(manager);
        case UART_ACTION_TRANSMIT:
            return uart_manager_notify_transmit_complete_handler(manager);
        default:
            return UART_ERR_OK;
    }
}