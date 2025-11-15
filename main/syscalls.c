#include "FreeRTOS.h"
#include "config.h"
#include "manager.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <time.h>

char* __env[1] = {0};
char** environ = __env;

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    _kill(status, -1);
    while (1) {
    }
}

int _read(int file, char* ptr, int len)
{
    (void)file;
    return len;
}

int _write(int file, char* ptr, int len)
{
    if (file == 1) {
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
#ifdef USE_LOG_TASK
            StreamBufferHandle_t log_stream_buffer =
                stream_buffer_manager_get(STREAM_BUFFER_TYPE_LOG);
            return xStreamBufferSend(log_stream_buffer,
                                     ptr,
                                     (size_t)len,
                                     pdMS_TO_TICKS(1000));
#else
            SemaphoreHandle_t log_mutex =
                semaphore_manager_get(SEMAPHORE_TYPE_LOG);
            if (xSemaphoreTake(log_mutex, portMAX_DELAY) == pdPASS) {
                CDC_Transmit_FS((uint8_t*)ptr, (uint16_t)len);
                xSemaphoreGive(log_mutex);
            }
        } else {
            CDC_Transmit_FS((uint8_t*)ptr, (uint16_t)len);
        }
#endif
            return len;
        }

        return 0;
    }

    int _close(int file)
    {
        (void)file;
        return -1;
    }

    int _fstat(int file, struct stat* st)
    {
        (void)file;
        st->st_mode = S_IFCHR;
        return 0;
    }

    int _isatty(int file)
    {
        (void)file;
        return 1;
    }

    int _lseek(int file, int ptr, int dir)
    {
        (void)file;
        (void)ptr;
        (void)dir;
        return 0;
    }

    int _open(char* path, int flags, ...)
    {
        (void)path;
        (void)flags;
        return -1;
    }

    int _wait(int* status)
    {
        (void)status;
        errno = ECHILD;
        return -1;
    }

    int _unlink(char* name)
    {
        (void)name;
        errno = ENOENT;
        return -1;
    }

    clock_t _times(struct tms * buf)
    {
        (void)buf;
        return -1;
    }

    int _stat(const char* file, struct stat* st)
    {
        (void)file;
        st->st_mode = S_IFCHR;
        return 0;
    }

    int _link(char* old, char* new)
    {
        (void)old;
        (void)new;
        errno = EMLINK;
        return -1;
    }

    int _fork(void)
    {
        errno = EAGAIN;
        return -1;
    }

    int _execve(char* name, char** argv, char** env)
    {
        (void)name;
        (void)argv;
        (void)env;
        errno = ENOMEM;
        return -1;
    }
