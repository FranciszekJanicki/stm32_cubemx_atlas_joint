#include "manager.h"

DEFINE_MANAGER(task, TaskType_t, TaskHandle_t, TASK_TYPE_NUM)
DEFINE_MANAGER(queue, QueueType_t, QueueHandle_t, QUEUE_TYPE_NUM)
DEFINE_MANAGER(stream_buffer,
               StreamBufferType_t,
               StreamBufferHandle_t,
               STREAM_BUFFER_TYPE_NUM)
DEFINE_MANAGER(semaphore,
               SemaphoreType_t,
               SemaphoreHandle_t,
               SEMAPHORE_TYPE_NUM)
DEFINE_MANAGER(timer, TimerType_t, TimerHandle_t, TIMER_TYPE_NUM);
