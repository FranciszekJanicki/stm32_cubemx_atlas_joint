#include "watchdog_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "queue.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_iwdg.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_wwdg.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

static char const* const TAG = "atlas_joint:watchdog_manager";

static inline bool watchdog_manager_reset_window_watchdog(
    watchdog_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_WWDG_Refresh(manager->config.window_watchdog) == HAL_OK;
}

static inline bool watchdog_manager_reset_independent_watchdog(
    watchdog_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    return HAL_IWDG_Refresh(manager->config.independent_watchdog) == HAL_OK;
}

static inline bool watchdog_manager_receive_watchdog_notify(
    watchdog_notify_t* notify)
{
    ATLAS_ASSERT(notify);

    return xTaskNotifyWait(0,
                           WATCHDOG_NOTIFY_ALL,
                           (uint32_t*)notify,
                           portMAX_DELAY) == pdPASS;
}

static atlas_err_t watchdog_manager_notify_refresh_timer_handler(
    watchdog_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    if (!manager->is_packet_alive || !manager->is_joint_alive ||
        !manager->is_system_alive) {
        return ATLAS_ERR_FAIL;
    }

    manager->is_packet_alive = false;
    manager->is_joint_alive = false;
    manager->is_system_alive = false;

    if (!watchdog_manager_reset_window_watchdog(manager)) {
        return ATLAS_ERR_FAIL;
    }

    if (!watchdog_manager_reset_independent_watchdog(manager)) {
        return ATLAS_ERR_FAIL;
    }

    return ATLAS_ERR_OK;
}

static atlas_err_t watchdog_manager_notify_packet_alive_handler(
    watchdog_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    manager->is_packet_alive = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t watchdog_manager_notify_joint_alive_handler(
    watchdog_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    manager->is_joint_alive = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t watchdog_manager_notify_system_alive_handler(
    watchdog_manager_t* manager)
{
    ATLAS_ASSERT(manager);
    ATLAS_LOG_FUNC(TAG);

    manager->is_system_alive = true;

    return ATLAS_ERR_OK;
}

static atlas_err_t watchdog_manager_notify_handler(watchdog_manager_t* manager,
                                                   watchdog_notify_t notify)
{
    ATLAS_ASSERT(manager);

    if ((notify & WATCHDOG_NOTIFY_WATCHDOG_TIMER) ==
        WATCHDOG_NOTIFY_WATCHDOG_TIMER) {
        ATLAS_RET_ON_ERR(
            watchdog_manager_notify_refresh_timer_handler(manager));
    }
    if ((notify & WATCHDOG_NOTIFY_JOINT_ALIVE) == WATCHDOG_NOTIFY_JOINT_ALIVE) {
        ATLAS_RET_ON_ERR(watchdog_manager_notify_joint_alive_handler(manager));
    }
    if ((notify & WATCHDOG_NOTIFY_SYSTEM_ALIVE) ==
        WATCHDOG_NOTIFY_SYSTEM_ALIVE) {
        ATLAS_RET_ON_ERR(watchdog_manager_notify_system_alive_handler(manager));
    }
    if ((notify & WATCHDOG_NOTIFY_PACKET_ALIVE) ==
        WATCHDOG_NOTIFY_PACKET_ALIVE) {
        ATLAS_RET_ON_ERR(watchdog_manager_notify_packet_alive_handler(manager));
    }

    return ATLAS_ERR_OK;
}

atlas_err_t watchdog_manager_process(watchdog_manager_t* manager)
{
    ATLAS_ASSERT(manager);

    watchdog_notify_t notify;
    if (watchdog_manager_receive_watchdog_notify(&notify)) {
        ATLAS_LOG_ON_ERR(TAG, watchdog_manager_notify_handler(manager, notify));
    }

    return ATLAS_ERR_OK;
}

atlas_err_t watchdog_manager_initialize(watchdog_manager_t* manager,
                                        watchdog_config_t const* config)
{
    ATLAS_ASSERT(manager && config);

    manager->config = *config;
    manager->is_packet_alive = false;
    manager->is_joint_alive = false;
    manager->is_system_alive = false;

    return ATLAS_ERR_OK;
}
