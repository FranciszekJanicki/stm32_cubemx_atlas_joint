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
    TASK_TYPE_JOINTS,
    TASK_TYPE_PACKET,
    TASK_TYPE_SAFETY,
    TASK_TYPE_UART,
    TASK_TYPE_NUM,
} TaskType_t;

typedef enum {
    QUEUE_TYPE_SYSTEM,
    QUEUE_TYPE_JOINTS,
    QUEUE_TYPE_PACKET,
    QUEUE_TYPE_SAFETY,
    QUEUE_TYPE_NUM,
} QueueType_t;

typedef enum {
    STREAM_BUFFER_TYPE_UART,
    STREAM_BUFFER_TYPE_NUM,
} StreamBufferType_t;

typedef enum {
    SEMAPHORE_TYPE_UART,
    SEMAPHORE_TYPE_JOINTS,
    SEMAPHORE_TYPE_NUM,
} SemaphoreType_t;

typedef enum {
    TIMER_TYPE_SYSTEM,
    TIMER_TYPE_NUM,
} TimerType_t;

DECLARE_HANDLE_MANAGER(task, TaskType_t, TaskHandle_t, TASK_TYPE_NUM)
DECLARE_HANDLE_MANAGER(queue, QueueType_t, QueueHandle_t, QUEUE_TYPE_NUM)
DECLARE_HANDLE_MANAGER(stream_buffer,
                       StreamBufferType_t,
                       StreamBufferHandle_t,
                       STREAM_BUFFER_TYPE_NUM)
DECLARE_HANDLE_MANAGER(semaphore, SemaphoreType_t, SemaphoreHandle_t, SEMAPHORE_TYPE_NUM)
DECLARE_HANDLE_MANAGER(timer, TimerType_t, TimerHandle_t, TIMER_TYPE_NUM);

#endif // COMMON_MANAGER_H