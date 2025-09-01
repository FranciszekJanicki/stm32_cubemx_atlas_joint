#include "atlas_joint.h"
#include "joint_task.h"
#include "log_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "task.h"
#include <string.h>

void atlas_joint_initialize(atlas_joint_config_t const* config)
{
    ATLAS_ASSERT(config);

    ATLAS_ERR_CHECK(log_task_initialize(&config->log_ctx));
    ATLAS_ERR_CHECK(joint_task_initialize(&config->joint_ctx));
    ATLAS_ERR_CHECK(packet_task_initialize(&config->packet_ctx));
    ATLAS_ERR_CHECK(system_task_initialize(&config->system_ctx));

    vTaskStartScheduler();
}
