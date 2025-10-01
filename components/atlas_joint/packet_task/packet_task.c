#include "packet_task.h"
#include "FreeRTOS.h"
#include "common.h"
#include "packet_manager.h"
#include "queue.h"
#include "task.h"
#include <stdint.h>

#define PACKET_TASK_STACK_DEPTH (10000U / sizeof(StackType_t))
#define PACKET_TASK_PRIORITY (1U)
#define PACKET_TASK_NAME ("atlas_joint:packet_task")

#define PACKET_QUEUE_ITEMS (10U)
#define PACKET_QUEUE_ITEM_SIZE (sizeof(packet_event_t))
#define PACKET_QUEUE_STORAGE_SIZE (PACKET_QUEUE_ITEMS * PACKET_QUEUE_ITEM_SIZE)

static void packet_task_func(void* ctx)
{
    packet_task_ctx_t* task_ctx = (packet_task_ctx_t*)ctx;

    packet_manager_t manager;
    ATLAS_LOG_ON_ERR(PACKET_TASK_NAME,
                     packet_manager_initialize(&manager, &task_ctx->config));

    while (1) {
        ATLAS_LOG_ON_ERR(PACKET_TASK_NAME, packet_manager_process(&manager));
        ATLAS_DELAY(10);
    }
}

static TaskHandle_t packet_task_create_task(packet_task_ctx_t* task_ctx)
{
    static StaticTask_t packet_task_buffer;
    static StackType_t packet_task_stack[PACKET_TASK_STACK_DEPTH];

    return xTaskCreateStatic(packet_task_func,
                             PACKET_TASK_NAME,
                             PACKET_TASK_STACK_DEPTH,
                             task_ctx,
                             PACKET_TASK_PRIORITY,
                             packet_task_stack,
                             &packet_task_buffer);
}

static QueueHandle_t packet_task_create_queue(void)
{
    static StaticQueue_t packet_queue_buffer;
    static uint8_t packet_queue_storage[PACKET_QUEUE_STORAGE_SIZE];

    return xQueueCreateStatic(PACKET_QUEUE_ITEMS,
                              PACKET_QUEUE_ITEM_SIZE,
                              packet_queue_storage,
                              &packet_queue_buffer);
}

#ifdef JOINT_PACKET_TEST
#define JOINT_PACKET_TEST_TIMER_NAME ("packet_test_timer")
#define JOINT_PACKET_TEST_TIMER_PERIOD (pdMS_TO_TICKS(1000))
#define JOINT_PACKET_TEST_TIMER_AUTORELOAD (pdTRUE)
#define JOINT_PACKET_TEST_TIMER_ID (NULL)

static void packet_task_joint_packet_test_timer_callback(TimerHandle_t timer)
{
    xTaskNotify(task_manager_get(TASK_TYPE_PACKET),
                PACKET_NOTIFY_SLAVE_SELECT,
                eSetBits);
}

static TimerHandle_t packet_task_create_joint_packet_test_timer(
    packet_task_ctx_t* task_ctx)
{
    static StaticTimer_t joint_packet_test_timer_buffer;

    return xTimerCreateStatic(JOINT_PACKET_TEST_TIMER_NAME,
                              JOINT_PACKET_TEST_TIMER_PERIOD,
                              JOINT_PACKET_TEST_TIMER_AUTORELOAD,
                              JOINT_PACKET_TEST_TIMER_ID,
                              packet_task_joint_packet_test_timer_callback,
                              &joint_packet_test_timer_buffer);
}
#endif

#ifdef ROBOT_PACKET_TEST
#define ROBOT_PACKET_TEST_TIMER_NAME ("packet_test_timer")
#define ROBOT_PACKET_TEST_TIMER_PERIOD (pdMS_TO_TICKS(1000))
#define ROBOT_PACKET_TEST_TIMER_AUTORELOAD (pdTRUE)
#define ROBOT_PACKET_TEST_TIMER_ID (NULL)

static void packet_task_robot_packet_test_timer_callback(TimerHandle_t timer)
{
    xTaskNotify(task_manager_get(TASK_TYPE_PACKET),
                PACKET_NOTIFY_SLAVE_SELECT,
                eSetBits);
}

static TimerHandle_t packet_task_create_robot_packet_test_timer(
    packet_task_ctx_t* task_ctx)
{
    static StaticTimer_t robot_packet_test_timer_buffer;

    return xTimerCreateStatic(ROBOT_PACKET_TEST_TIMER_NAME,
                              ROBOT_PACKET_TEST_TIMER_PERIOD,
                              ROBOT_PACKET_TEST_TIMER_AUTORELOAD,
                              ROBOT_PACKET_TEST_TIMER_ID,
                              packet_task_robot_packet_test_timer_callback,
                              &robot_packet_test_timer_buffer);
}
#endif

atlas_err_t packet_task_initialize(packet_task_ctx_t* task_ctx)
{
    ATLAS_ASSERT(task_ctx);

#ifdef JOINT_PACKET_TEST
    TimerHandle_t joint_packet_test_timer =
        packet_task_create_joint_packet_test_timer(task_ctx);
    if (joint_packet_test_timer == NULL) {
        return ATLAS_ERR_FAIL;
    }

    timer_manager_set(TIMER_TYPE_JOINT_PACKET_TEST, joint_packet_test_timer);
#endif

#ifdef ROBOT_PACKET_TEST
    TimerHandle_t robot_packet_test_timer =
        packet_task_create_robot_packet_test_timer(task_ctx);
    if (robot_packet_test_timer == NULL) {
        return ATLAS_ERR_FAIL;
    }

    timer_manager_set(TIMER_TYPE_ROBOT_PACKET_TEST, robot_packet_test_timer);
#endif

    QueueHandle_t packet_queue = packet_task_create_queue();
    if (packet_queue == NULL) {
        return ATLAS_ERR_FAIL;
    }

    queue_manager_set(QUEUE_TYPE_PACKET, packet_queue);

    TaskHandle_t packet_task = packet_task_create_task(task_ctx);
    if (packet_task == NULL) {
        return ATLAS_ERR_FAIL;
    }

    task_manager_set(TASK_TYPE_PACKET, packet_task);

    return ATLAS_ERR_OK;
}

void packet_task_slave_select_callback(void)
{
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_PACKET),
                       PACKET_NOTIFY_SLAVE_SELECT,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

void packet_task_transfer_complete_callback(void)
{
    BaseType_t task_woken = pdFALSE;
    xTaskNotifyFromISR(task_manager_get(TASK_TYPE_PACKET),
                       PACKET_NOTIFY_TRANSFER_COMPLETE,
                       eSetBits,
                       &task_woken);

    portYIELD_FROM_ISR(task_woken);
}

#undef PACKET_TASK_STACK_DEPTH
#undef PACKET_TASK_PRIORITY
#undef PACKET_TASK_NAME

#undef PACKET_QUEUE_ITEMS
#undef PACKET_QUEUE_ITEM_SIZE
#undef PACKET_QUEUE_STORAGE_SIZE