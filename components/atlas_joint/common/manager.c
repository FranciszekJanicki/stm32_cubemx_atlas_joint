#include "manager.h"

DEFINE_HANDLE_MANAGER(task, TaskType_t, TaskHandle_t, TASK_TYPE_NUM)
DEFINE_HANDLE_MANAGER(queue, QueueType_t, QueueHandle_t, QUEUE_TYPE_NUM)
DEFINE_HANDLE_MANAGER(stream_buffer,
                      StreamBufferType_t,
                      StreamBufferHandle_t,
                      STREAM_BUFFER_TYPE_NUM)
DEFINE_HANDLE_MANAGER(semaphore, SemaphoreType_t, SemaphoreHandle_t, SEMAPHORE_TYPE_NUM)
DEFINE_HANDLE_MANAGER(timer, TimerType_t, TimerHandle_t, TIMER_TYPE_NUM);
