#include "atlas_joint.h"
#include "joint_task.h"
#include "log_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "task.h"
#ifdef USE_WATCHDOG_TASK
#include "watchdog_task.h"
#endif
#include <string.h>

bool volatile is_kernel_started = false;

void atlas_joint_initialize(atlas_joint_config_t const* config)
{
    ATLAS_ASSERT(config);
#ifdef USE_WATCHDOG_TASK
    ATLAS_ERR_CHECK(watchdog_task_initialize(&config->watchdog_ctx));
#endif
#ifdef USE_LOG_TASK
    ATLAS_ERR_CHECK(log_task_initialize(&config->log_ctx));
#endif
    ATLAS_ERR_CHECK(packet_task_initialize(&config->packet_ctx));
    ATLAS_ERR_CHECK(system_task_initialize(&config->system_ctx));
    ATLAS_ERR_CHECK(joint_task_initialize(&config->joint_ctx));

    is_kernel_started = true;

    vTaskStartScheduler();
}
