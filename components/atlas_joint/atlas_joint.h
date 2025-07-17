#ifndef ATLAS_JOINT_ATLAS_JOINT_H
#define ATLAS_JOINT_ATLAS_JOINT_H

#include "joint_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "uart_task.h"

typedef struct {
    uart_task_ctx_t uart_task_ctx;
    packet_task_ctx_t packet_task_ctx;
    joint_task_ctx_t joint_task_ctx;
} atlas_joint_config_t;

void atlas_joint_initialize(atlas_joint_config_t const* config);

#endif // ATLAS_ROB_ATLAS_ROB_H
