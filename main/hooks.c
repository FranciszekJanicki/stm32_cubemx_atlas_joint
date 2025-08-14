#include "FreeRTOS.h"
#include "common.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include "usart.h"
#include <string.h>

__attribute__((used)) void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                                         signed char* pcTaskName)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)"Overflow: ", 10, 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)pcTaskName, strlen(pcTaskName), 100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);

    ATLAS_PANIC();
}

__attribute__((used)) void vApplicationMallocFailedHook(void)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)"vApplicationMallocFailedHook", 28, 100);

    ATLAS_PANIC();
}