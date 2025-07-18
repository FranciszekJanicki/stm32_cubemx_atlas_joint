#ifndef SYSTEM_TASK_SYSTEM_TASK_H
#define SYSTEM_TASK_SYSTEM_TASK_H

#include "common.h"
#include "system_manager.h"

typedef struct {
    system_config_t config;
} system_task_ctx_t;

atlas_err_t system_task_initialize(system_task_ctx_t* task_ctx);

#endif // SYSTEM_TASK_SYSTEM_TASK_H