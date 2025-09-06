#ifndef WATCHDOG_TASK_WATCHDOG_TASK_H
#define WATCHDOG_TASK_WATCHDOG_TASK_H

#include "common.h"
#include "watchdog_manager.h"

typedef struct {
    watchdog_config_t config;
} watchdog_task_ctx_t;

atlas_err_t watchdog_task_initialize(watchdog_task_ctx_t* task_ctx);

void watchdog_task_refresh_timer_callback(void);

#endif // WATCHDOG_TASK_WATCHDOG_TASK_H