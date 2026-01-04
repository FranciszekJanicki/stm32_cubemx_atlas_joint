#ifndef SYSTEM_TASK_SYSTEM_MANAGER_H
#define SYSTEM_TASK_SYSTEM_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "stm32f411xe.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    GPIO_TypeDef* delta_timer_gpio;
    uint16_t delta_timer_pin;
#ifndef USE_WATCHDOG_TASK
    TIM_HandleTypeDef* refresh_timer;
    WWDG_HandleTypeDef* window_watchdog;
    IWDG_HandleTypeDef* independent_watchdog;
#endif
} system_config_t;

typedef struct {
    bool is_packet_running;
    bool is_joint_running;

    system_config_t config;
} system_manager_t;

atlas_err_t system_manager_initialize(system_manager_t* manager,
                                      system_config_t const* config);
atlas_err_t system_manager_process(system_manager_t* manager);

#endif // SYSTEM_TASK_SYSTEM_MANAGER_H
