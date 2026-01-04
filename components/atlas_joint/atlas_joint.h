#ifndef ATLAS_JOINT_ATLAS_JOINT_H
#define ATLAS_JOINT_ATLAS_JOINT_H

#include "common.h"
#include "joint_task.h"
#ifdef USE_LOG_TASK
#include "log_task.h"
#endif
#include "packet_task.h"
#include "system_task.h"
#ifdef USE_WATCHDOG_TASK
#include "watchdog_task.h"
#endif

typedef struct {
    system_task_ctx_t system_ctx;
#ifdef USE_LOG_TASK
    log_task_ctx_t log_ctx;
#endif
#ifdef USE_WATCHDOG_TASK
    watchdog_task_ctx_t watchdog_ctx;
#endif
    packet_task_ctx_t packet_ctx;
    joint_task_ctx_t joint_ctx;
} atlas_joint_config_t;

extern bool volatile is_kernel_started;

void atlas_joint_initialize(atlas_joint_config_t const* config);

#endif // ATLAS_JOINT_ATLAS_JOINT_H
