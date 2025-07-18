#include "joint_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "joint_manager.h"
#include "queue.h"
#include "task.h"
#include <stdint.h>

#define JOINT_TASK_STACK_DEPTH (5000U / sizeof(StackType_t))
#define JOINT_TASK_PRIORITY (1U)
#define JOINT_TASK_NAME ("joint_task")

#define JOINT_QUEUE_ITEMS (10U)
#define JOINT_QUEUE_ITEM_SIZE (sizeof(joint_event_t))
#define JOINT_QUEUE_STORAGE_SIZE (JOINT_QUEUE_ITEMS * JOINT_QUEUE_ITEM_SIZE)

static void joint_task_func(void* ctx)
{
    joint_task_ctx_t* task_ctx = (joint_task_ctx_t*)ctx;

    joint_manager_t manager;
    ATLAS_LOG_ON_ERR(JOINT_TASK_NAME,
                     joint_manager_initialize(&manager,
                                              &task_ctx->config,
                                              &task_ctx->parameters));

    while (1) {
        ATLAS_LOG_ON_ERR(JOINT_TASK_NAME, joint_manager_process(&manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static TaskHandle_t joint_task_create_task(joint_task_ctx_t* task_ctx)
{
    static StaticTask_t joint_task_buffer;
    static StackType_t joint_task_stack[JOINT_TASK_STACK_DEPTH];

    return xTaskCreateStatic(joint_task_func,
                             JOINT_TASK_NAME,
                             JOINT_TASK_STACK_DEPTH,
                             task_ctx,
                             JOINT_TASK_PRIORITY,
                             joint_task_stack,
                             &joint_task_buffer);
}

static QueueHandle_t joint_task_create_queue(void)
{
    static StaticQueue_t joint_queue_buffer;
    static uint8_t joint_queue_storage[JOINT_QUEUE_STORAGE_SIZE];

    return xQueueCreateStatic(JOINT_QUEUE_ITEMS,
                              JOINT_QUEUE_ITEM_SIZE,
                              joint_queue_storage,
                              &joint_queue_buffer);
}

atlas_err_t joint_task_initialize(joint_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

    QueueHandle_t joint_queue = joint_task_create_queue();
    if (joint_queue == NULL) {
        return ATLAS_ERR_FAIL;
    }

    TaskHandle_t joint_task = joint_task_create_task(task_ctx);
    if (joint_task == NULL) {
        return ATLAS_ERR_FAIL;
    }

    task_manager_set(TASK_TYPE_JOINT, joint_task);
    queue_manager_set(QUEUE_TYPE_JOINT, joint_queue);

    return ATLAS_ERR_OK;
}

void joint_delta_timer_callback(void)
{
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_JOINT),
                       JOINT_NOTIFY_DELTA_TIMER,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

void joint_pwm_pulse_callback(void)
{
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_JOINT),
                       JOINT_NOTIFY_PWM_PULSE,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

#undef JOINT_TASK_STACK_DEPTH
#undef JOINT_TASK_PRIORITY
#undef JOINT_TASK_NAME

#undef JOINT_QUEUE_ITEMS
#undef JOINT_QUEUE_ITEM_SIZE
#undef JOINT_QUEUE_STORAGE_SIZE