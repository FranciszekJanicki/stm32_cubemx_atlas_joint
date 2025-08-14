#ifndef SYSTEM_TASK_SYSTEM_MANAGER_H
#define SYSTEM_TASK_SYSTEM_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    atlas_joint_num_t num;
    RTC_HandleTypeDef* timestamp_rtc;
#ifdef PACKET_TEST
    TIM_HandleTypeDef* delta_timer;       
    TIM_HandleTypeDef* packet_ready_timer;
    #endif
} system_config_t;

typedef struct {
    bool is_running;
    bool is_joint_running;
    bool is_packet_running;

    atlas_timestamp_t current_timestamp;
    atlas_timestamp_t start_timestamp;

    atlas_joint_measure_t joint_measure;
    atlas_joint_reference_t joint_reference;

    system_config_t config;
} system_manager_t;

atlas_err_t system_manager_initialize(system_manager_t* manager,
                                      system_config_t const* config);
atlas_err_t system_manager_process(system_manager_t* manager);

#endif // SYSTEM_TASK_SYSTEM_MANAGER_H