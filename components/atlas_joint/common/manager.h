#ifndef COMMON_MANAGER_H
#define COMMON_MANAGER_H

#include "FreeRTOS.h"
#include "handle_manager.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"
#include "timers.h"

typedef enum {
    TASK_TYPE_SYSTEM,
    TASK_TYPE_JOINT,
    TASK_TYPE_PACKET,
#ifdef USE_LOG_TASK
    TASK_TYPE_LOG,
#endif
#ifdef USE_WATCHDOG_TASK
    TASK_TYPE_WATCHDOG,
#endif
    TASK_TYPE_NUM,
} TaskType_t;

typedef enum {
    QUEUE_TYPE_SYSTEM,
    QUEUE_TYPE_JOINT,
    QUEUE_TYPE_PACKET,
    QUEUE_TYPE_NUM,
} QueueType_t;

typedef enum {
#ifdef USE_LOG_TASK
    STREAM_BUFFER_TYPE_LOG,
#endif
    STREAM_BUFFER_TYPE_NUM,
} StreamBufferType_t;

typedef enum {
#ifndef USE_LOG_TASK
    SEMAPHORE_TYPE_LOG,
#endif
    SEMAPHORE_TYPE_JOINT,
    SEMAPHORE_TYPE_NUM,
} SemaphoreType_t;

typedef enum {
    TIMER_TYPE_JOINT_PACKET_TEST,
    TIMER_TYPE_ROBOT_PACKET_TEST,
    TIMER_TYPE_DELTA_TEST,
    TIMER_TYPE_NUM,
} TimerType_t;

DECLARE_HANDLE_MANAGER(task, TaskType_t, TaskHandle_t, TASK_TYPE_NUM)
DECLARE_HANDLE_MANAGER(queue, QueueType_t, QueueHandle_t, QUEUE_TYPE_NUM)
DECLARE_HANDLE_MANAGER(stream_buffer,
                       StreamBufferType_t,
                       StreamBufferHandle_t,
                       STREAM_BUFFER_TYPE_NUM)
DECLARE_HANDLE_MANAGER(semaphore,
                       SemaphoreType_t,
                       SemaphoreHandle_t,
                       SEMAPHORE_TYPE_NUM)
DECLARE_HANDLE_MANAGER(timer, TimerType_t, TimerHandle_t, TIMER_TYPE_NUM);

#endif // COMMON_MANAGER_H
