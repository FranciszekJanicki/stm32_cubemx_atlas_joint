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
    GPIO_TypeDef* data_ready_gpio;
    uint16_t data_ready_pin;

    GPIO_TypeDef* slave_select_gpio;
    uint16_t slave_select_pin;

    SPI_HandleTypeDef* packet_spi_bus;
} packet_config_t;

typedef struct {
    bool is_running;
    bool is_waiting_for_transmit;
    bool is_transmit_finished;
    bool is_transmit_sent;

    packet_config_t config;

    uint8_t receive_buffer[ATLAS_JOINT_PACKET_SIZE];
    uint8_t transmit_buffer[ATLAS_ROBOT_PACKET_SIZE];
} packet_manager_t;

atlas_err_t packet_manager_initialize(packet_manager_t* manager,

                                      packet_config_t const* config);
atlas_err_t packet_manager_process(packet_manager_t* manager);

#endif // PACKET_TASK_PACKET_MANAGER_H