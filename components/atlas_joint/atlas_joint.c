#include "atlas_joint.h"
#include "joint_task.h"
#include "packet_task.h"
#include "system_task.h"
#include "task.h"
#include "uart_task.h"
#include <string.h>

void atlas_joint_initialize(atlas_joint_config_t const* config)
{
    ATLAS_ASSERT(config);

    ATLAS_ERR_CHECK(uart_task_initialize(&config->uart_ctx));
    ATLAS_ERR_CHECK(system_task_initialize(&config->system_ctx));
    ATLAS_ERR_CHECK(uart_task_initialize(&config->uart_ctx));
    ATLAS_ERR_CHECK(joint_task_initialize(&config->joint_ctx));
    ATLAS_ERR_CHECK(packet_task_initialize(&config->packet_ctx));

    vTaskStartScheduler();
}

#undef UART_TASK_STACK_DEPTH
#undef UART_TASK_PRIORITY
#undef UART_TASK_NAME

#undef UART_BUFFER_STORAGE_SIZE

#undef UART_STREAM_BUFFER_STORAGE_SIZE
#undef UART_STREAM_BUFFER_TRIGGER
