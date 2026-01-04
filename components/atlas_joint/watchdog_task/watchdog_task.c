#include "watchdog_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "task.h"
#include "watchdog_manager.h"
#include <stdint.h>

#define WATCHDOG_TASK_STACK_DEPTH (6000U / sizeof(StackType_t))
#define WATCHDOG_TASK_PRIORITY (1U)
#define WATCHDOG_TASK_NAME ("atlas_joint:watchdog_task")

#define WATCHDOG_QUEUE_ITEMS (10U)
#define WATCHDOG_QUEUE_ITEM_SIZE (sizeof(watchdog_event_t))
#define WATCHDOG_QUEUE_STORAGE_SIZE \
    (WATCHDOG_QUEUE_ITEMS * WATCHDOG_QUEUE_ITEM_SIZE)

static void watchdog_task_func(void* ctx)
{
    watchdog_task_ctx_t* task_ctx = (watchdog_task_ctx_t*)ctx;

    watchdog_manager_t manager;
    ATLAS_LOG_ON_ERR(WATCHDOG_TASK_NAME,
                     watchdog_manager_initialize(&manager, &task_ctx->config));

    while (1) {
        ATLAS_LOG_ON_ERR(WATCHDOG_TASK_NAME,
                         watchdog_manager_process(&manager));
        ATLAS_DELAY(10);
    }
}

static TaskHandle_t watchdog_task_create_task(watchdog_task_ctx_t* task_ctx)
{
    static StaticTask_t watchdog_task_buffer;
    static StackType_t watchdog_task_stack[WATCHDOG_TASK_STACK_DEPTH];

    return xTaskCreateStatic(watchdog_task_func,
                             WATCHDOG_TASK_NAME,
                             WATCHDOG_TASK_STACK_DEPTH,
                             task_ctx,
                             WATCHDOG_TASK_PRIORITY,
                             watchdog_task_stack,
                             &watchdog_task_buffer);
}

void watchdog_task_refresh_timer_callback(void)
{
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_WATCHDOG),
                       WATCHDOG_NOTIFY_REFRESH_TIMER,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

atlas_err_t watchdog_task_start_refresh_timer(watchdog_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    return HAL_TIM_Base_Start_IT(task_ctx->config.refresh_timer) == HAL_OK
               ? ATLAS_ERR_OK
               : ATLAS_ERR_FAIL;
}

atlas_err_t watchdog_task_stop_refresh_timer(watchdog_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    return HAL_TIM_Base_Stop_IT(task_ctx->config.refresh_timer) == HAL_OK
               ? ATLAS_ERR_OK
               : ATLAS_ERR_FAIL;
}

atlas_err_t watchdog_task_initialize(watchdog_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    ATLAS_RET_ON_ERR(watchdog_task_start_refresh_timer(task_ctx));

    TaskHandle_t watchdog_task = watchdog_task_create_task(task_ctx);
    if (watchdog_task == NULL) {
        return ATLAS_ERR_FAIL;
    }

    task_manager_set(TASK_TYPE_WATCHDOG, watchdog_task);

    return ATLAS_ERR_OK;
}

#undef WATCHDOG_TASK_STACK_DEPTH
#undef WATCHDOG_TASK_PRIORITY
#undef WATCHDOG_TASK_NAME

#undef WATCHDOG_QUEUE_ITEMS
#undef WATCHDOG_QUEUE_ITEM_SIZE
#undef WATCHDOG_QUEUE_STORAGE_SIZE
