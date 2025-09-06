#ifndef WATCHDOG_TASK_WATCHDOG_MANAGER_H
#define WATCHDOG_TASK_WATCHDOG_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    TIM_HandleTypeDef* refresh_timer;
    WWDG_HandleTypeDef* window_watchdog;
    IWDG_HandleTypeDef* independent_watchdog;
} watchdog_config_t;

typedef struct {
    bool is_joint_alive;
    bool is_system_alive;
    bool is_packet_alive;

    watchdog_config_t config;
} watchdog_manager_t;

atlas_err_t watchdog_manager_initialize(watchdog_manager_t* manager,
                                        watchdog_config_t const* config);
atlas_err_t watchdog_manager_process(watchdog_manager_t* manager);

#endif // WATCHDOG_TASK_WATCHDOG_MANAGER_H