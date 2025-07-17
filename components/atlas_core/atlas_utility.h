#ifndef ATLAS_CORE_ATLAS_UTILITY_H
#define ATLAS_CORE_ATLAS_UTILITY_H

#include "FreeRTOS.h"
#include "atlas_log.h"
#include "task.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#define DEBUG

#define ATLAS_PANIC()             \
    do {                          \
        taskDISABLE_INTERRUPTS(); \
        while (1)                 \
            ;                     \
    } while (0)

#define ATLAS_RET_ON_ERR(ERR)      \
    do {                           \
        atlas_err_t err = (ERR);   \
        if (err != ATLAS_ERR_OK) { \
            return err;            \
        }                          \
    } while (0)

#ifdef DEBUG

#define ATLAS_LOG(TAG, FMT, ...) atlas_log("[%s] " FMT "\n\r", TAG, ##__VA_ARGS__)

#define ATLAS_LOG_FUNC(TAG) ATLAS_LOG(TAG, "%s", __func__)

#define ATLAS_LOG_ON_ERR(TAG, ERR)                          \
    do {                                                    \
        atlas_err_t err = (ERR);                            \
        if (err != ATLAS_ERR_OK) {                          \
            ATLAS_LOG(TAG, "%s", atlas_err_to_string(err)); \
        }                                                   \
    } while (0)

#define ATLAS_ASSERT(EXPR)                                                                        \
    do {                                                                                          \
        if (!(EXPR)) {                                                                            \
            ATLAS_LOG("ATLAS_ASSERT", "%s: %d: Assertion failed: %s", __FILE__, __LINE__, #EXPR); \
            vTaskDelay(100);                                                                      \
            ATLAS_PANIC();                                                                        \
        }                                                                                         \
    } while (0)

#define ATLAS_ERR_CHECK(ERR) ATLAS_ASSERT((ERR) == ATLAS_ERR_OK)

#else

#define ATLAS_LOG(TAG, FMT, ...) \
    do {                         \
    } while (0)

#define ATLAS_LOG_FUNC(TAG) \
    do {                    \
    } while (0)

#define ATLAS_LOG_ON_ERR(TAG, ERR) \
    do {                           \
    } while (0)

#define ATLAS_ASSERT(EXPR) \
    do {                   \
        (void)(EXPR);      \
    } while (0)

#define ATLAS_ERR_CHECK(ERR) \
    do {                     \
    } while (0)

#endif // DEBUG

#endif // ATLAS_CORE_ATLAS_UTILITY_H
