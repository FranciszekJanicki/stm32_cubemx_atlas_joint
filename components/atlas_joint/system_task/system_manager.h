#ifndef SYSTEM_TASK_SYSTEM_MANAGER_H
#define SYSTEM_TASK_SYSTEM_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "task.h"
#include "timers.h"
#include <stdbool.h>

typedef struct {
    bool is_running;
    bool is_joints_ready;
    bool is_packet_ready;
    bool is_safety_ready;
} system_manager_t;

atlas_err_t system_manager_initialize(system_manager_t* manager);
atlas_err_t system_manager_process(system_manager_t* manager);

#endif // SYSTEM_TASK_SYSTEM_MANAGER_H