#ifndef PACKET_TASK_PACKET_MANAGER_H
#define PACKET_TASK_PACKET_MANAGER_H

#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <stdbool.h>

typedef struct {
    GPIO_TypeDef* robot_packet_ready_gpio;
    uint16_t robot_packet_ready_pin;

    GPIO_TypeDef* joint_packet_ready_gpio;
    uint16_t joint_packet_ready_pin;

    SPI_HandleTypeDef* packet_spi_bus;
#ifdef PACKET_TEST
    TIM_HandleTypeDef* joint_packet_ready_timer;
#endif
} packet_config_t;

typedef struct {
    bool is_running;

    packet_config_t config;
} packet_manager_t;

atlas_err_t packet_manager_initialize(packet_manager_t* manager,
                                      packet_config_t const* config);
atlas_err_t packet_manager_process(packet_manager_t* manager);

#endif // PACKET_TASK_PACKET_MANAGER_H