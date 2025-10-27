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

#define TRANSFER_BUFFER_SIZE                           \
    (ATLAS_JOINT_PACKET_SIZE > ATLAS_ROBOT_PACKET_SIZE \
         ? ATLAS_JOINT_PACKET_SIZE                     \
         : ATLAS_ROBOT_PACKET_SIZE)
#define RECEIVE_BUFFER_SIZE (TRANSFER_BUFFER_SIZE)
#define TRANSMIT_BUFFER_SIZE (TRANSFER_BUFFER_SIZE)

typedef struct {
    bool is_running;
    bool is_transfer_pending;
    bool is_transfer_complete;

    packet_config_t config;

    uint8_t receive_buffer[RECEIVE_BUFFER_SIZE];
    uint8_t transmit_buffer[TRANSMIT_BUFFER_SIZE];
} packet_manager_t;

atlas_err_t packet_manager_initialize(packet_manager_t* manager,

                                      packet_config_t const* config);
atlas_err_t packet_manager_process(packet_manager_t* manager);

#endif // PACKET_TASK_PACKET_MANAGER_H
