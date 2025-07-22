#include "FreeRTOS.h"
#include "manager.h"
#include "semphr.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "stream_buffer.h"
#include "task.h"
#include "usart.h"

__attribute__((used)) int _write(int file, char* ptr, int len)
{
    StreamBufferHandle_t uart_stream_buffer =
        stream_buffer_manager_get(STREAM_BUFFER_TYPE_UART);
    SemaphoreHandle_t uart_mutex = semaphore_manager_get(SEMAPHORE_TYPE_UART);

    size_t written = 0U;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100))) {
        // written = xStreamBufferSend(uart_stream_buffer, ptr, len, 0);
        HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, len);
        xSemaphoreGive(uart_mutex);
    }

    return written;
}