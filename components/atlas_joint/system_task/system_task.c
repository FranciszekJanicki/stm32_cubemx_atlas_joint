#include "system_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "system_manager.h"
#include "task.h"
#include <stdint.h>

#define SYSTEM_TASK_STACK_DEPTH (5000U / sizeof(StackType_t))
#define SYSTEM_TASK_PRIORITY (1U)
#define SYSTEM_TASK_NAME ("system_task")

#define SYSTEM_QUEUE_ITEMS (10U)
#define SYSTEM_QUEUE_ITEM_SIZE (sizeof(system_event_t))
#define SYSTEM_QUEUE_STORAGE_SIZE (SYSTEM_QUEUE_ITEMS * SYSTEM_QUEUE_ITEM_SIZE)

#define SYSTEM_TIMER_NAME ("system_timer")
#define SYSTEM_TIMER_PERIOD_MS (10000U)
#define SYSTEM_TIMER_PERIOD_TICKS (pdMS_TO_TICKS(SYSTEM_TIMER_PERIOD_MS))
#define SYSTEM_TIMER_ID (NULL)
#define SYSTEM_TIMER_AUTORELOAD (pdFALSE)

static void system_task_func(void*)
{
    system_manager_t manager;
    ATLAS_LOG_ON_ERR(SYSTEM_TASK_NAME, system_manager_initialize(&manager));

    while (1) {
        ATLAS_LOG_ON_ERR(SYSTEM_TASK_NAME, system_manager_process(&manager));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void system_timer_callback(TimerHandle_t timer)
{
    BaseType_t task_woken = pdFALSE;

    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_SYSTEM),
                       SYSTEM_NOTIFY_RETRY_TIMER,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

static TaskHandle_t system_task_create_task(void)
{
    static StaticTask_t system_task_buffer;
    static StackType_t system_task_stack[SYSTEM_TASK_STACK_DEPTH];

    return xTaskCreateStatic(system_task_func,
                             SYSTEM_TASK_NAME,
                             SYSTEM_TASK_STACK_DEPTH,
                             NULL,
                             SYSTEM_TASK_PRIORITY,
                             system_task_stack,
                             &system_task_buffer);
}

static QueueHandle_t system_task_create_queue(void)
{
    static StaticQueue_t system_queue_buffer;
    static uint8_t system_queue_storage[SYSTEM_QUEUE_STORAGE_SIZE];

    return xQueueCreateStatic(SYSTEM_QUEUE_ITEMS,
                              SYSTEM_QUEUE_ITEM_SIZE,
                              system_queue_storage,
                              &system_queue_buffer);
}

static TimerHandle_t system_task_create_timer(void)
{
    static StaticTimer_t system_timer_buffer;

    return xTimerCreateStatic(SYSTEM_TIMER_NAME,
                              SYSTEM_TIMER_PERIOD_TICKS,
                              SYSTEM_TIMER_AUTORELOAD,
                              SYSTEM_TIMER_ID,
                              system_timer_callback,
                              &system_timer_buffer);
}

void system_task_initialize(void)
{
    queue_manager_set(QUEUE_TYPE_SYSTEM, system_task_create_queue());
    timer_manager_set(TIMER_TYPE_SYSTEM, system_task_create_timer());
    task_manager_set(TASK_TYPE_SYSTEM, system_task_create_task());
}

#undef SYSTEM_TASK_STACK_DEPTH
#undef SYSTEM_TASK_PRIORITY
#undef SYSTEM_TASK_NAME

#undef SYSTEM_QUEUE_ITEMS
#undef SYSTEM_QUEUE_ITEM_SIZE
#undef SYSTEM_QUEUE_STORAGE_SIZE

#undef SYSTEM_TIMER_NAME
#undef SYSTEM_TIMER_PERIOD_MS
#undef SYSTEM_TIMER_PERIOD_TICKS
#undef SYSTEM_TIMER_ID
#undef SYSTEM_TIMER_AUTORELOAD