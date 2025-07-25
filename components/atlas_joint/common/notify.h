#ifndef COMMON_NOTIFY_H
#define COMMON_NOTIFY_H

#include "bus_task.h"

typedef enum {
    SYSTEM_NOTIFY_RETRY_TIMER = (1 << 0),
    SYSTEM_NOTIFY_JOINT_READY = (1 << 1),
    SYSTEM_NOTIFY_JOINT_FAULT = (1 << 2),
    SYSTEM_NOTIFY_PACKET_READY = (1 << 3),
    SYSTEM_NOTIFY_ALL =
        (SYSTEM_NOTIFY_RETRY_TIMER | SYSTEM_NOTIFY_JOINT_READY |
         SYSTEM_NOTIFY_JOINT_FAULT | SYSTEM_NOTIFY_PACKET_READY),
} system_notify_t;

typedef enum {
    JOINT_NOTIFY_DELTA_TIMER = (1 << 0),
    JOINT_NOTIFY_PWM_PULSE = (1 << 1),
    JOINT_NOTIFY_ALL = (JOINT_NOTIFY_DELTA_TIMER | JOINT_NOTIFY_PWM_PULSE),
} joint_notify_t;

typedef enum {
    PACKET_NOTIFY_JOINT_PACKET_READY = (1 << 0),
    PACKET_NOTIFY_ALL = (PACKET_NOTIFY_JOINT_PACKET_READY),
} packet_notify_t;

typedef enum {
    UART_NOTIFY_START = BUS_ACTION_TRANSMIT,
    UART_NOTIFY_STOP = BUS_NOTIFY_STOP,
    UART_NOTIFY_TRANSMIT_DONE = BUS_NOTIFY_TRANSMIT_DONE,
    UART_NOTIFY_ALL =
        (UART_NOTIFY_START | UART_NOTIFY_STOP | UART_NOTIFY_TRANSMIT_DONE),
} uart_notify_t;

#endif // COMMON_NOTIFY_H
