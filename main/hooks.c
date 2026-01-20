#include "FreeRTOS.h"
#include "common.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <string.h>

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName)
{
    atlas_log("Stack overflow on %s stack \n\r", pcTaskName);

    ATLAS_PANIC();
}
