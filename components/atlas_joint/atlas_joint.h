#ifndef ATLAS_JOINT_ATLAS_JOINT_H
#define ATLAS_JOINT_ATLAS_JOINT_H

#include "joint_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "uart_task.h"

typedef struct {
    system_task_ctx_t system_ctx;
    uart_task_ctx_t uart_ctx;
    packet_task_ctx_t packet_ctx;
    joint_task_ctx_t joint_ctx;
} atlas_joint_config_t;

void atlas_joint_initialize(atlas_joint_config_t const* config);

#endif // ATLAS_JOINT_ATLAS_JOINT_H
