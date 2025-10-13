#ifndef SYSTEM_TASK_SYSTEM_MANAGER_H
#define SYSTEM_TASK_SYSTEM_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    atlas_joint_num_t joint_num;

    RTC_HandleTypeDef* timestamp_rtc;

    GPIO_TypeDef* delta_timer_gpio;
    uint16_t delta_timer_pin;
} system_config_t;

typedef struct {
    bool is_packet_ready;
    bool is_packet_running;
    bool is_joint_ready;
    bool is_joint_running;
    bool is_joint_start_pending;

    atlas_timestamp_t current_timestamp;
    atlas_timestamp_t startup_timestamp;

    atlas_joint_measure_t joint_measure;
    atlas_joint_reference_t joint_reference;

    system_config_t config;
} system_manager_t;

atlas_err_t system_manager_initialize(system_manager_t* manager,
                                      system_config_t const* config);
atlas_err_t system_manager_process(system_manager_t* manager);

#endif // SYSTEM_TASK_SYSTEM_MANAGER_H