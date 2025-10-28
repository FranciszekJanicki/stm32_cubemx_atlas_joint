#include "FreeRTOS.h"
#include "atlas_joint.h"
#include "common.h"
#include "config.h"
#include "iwdg.h"
#include "log_task.h"
#include "packet_task.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "system_task.h"
#include "wwdg.h"

static inline void joint_slave_select_debounce_timer_callback(void)
{
    static uint8_t debounce_counter = 0U;

    if (HAL_GPIO_ReadPin(JOINT_SLAVE_SELECT_GPIO, JOINT_SLAVE_SELECT_PIN) ==
        GPIO_PIN_RESET) {
        if (debounce_counter++ >= 3U) {
            debounce_counter = 0U;

            packet_task_slave_select_callback();
            HAL_TIM_Base_Stop_IT(JOINT_SLAVE_SELECT_DEBOUNCE_TIMER);
        }
    }
}

static inline void joint_delta_elapsed_debounce_timer_callback(void)
{
    static uint8_t debounce_counter = 0U;

    if (HAL_GPIO_ReadPin(JOINT_DELTA_ELAPSED_GPIO, JOINT_DELTA_ELAPSED_PIN) ==
        GPIO_PIN_SET) {
        if (debounce_counter++ >= 3U) {
            debounce_counter = 0U;

            HAL_TIM_Base_Stop_IT(JOINT_DELTA_ELAPSED_DEBOUNCE_TIMER);
            joint_task_delta_elapsed_callback();
        }
    }
}

static inline void watchdog_refresh_timer_callback(void)
{
#ifdef USE_WATCHDOG_TASK
    if (is_kernel_started) {
        watchdog_task_refresh_timer_callback();
    } else {
        HAL_WWDG_Refresh(WINDOW_WATCHDOG);
        HAL_IWDG_Refresh(INDEPENDENT_WATCHDOG);
    }
#else
    HAL_WWDG_Refresh(WINDOW_WATCHDOG);
    HAL_IWDG_Refresh(INDEPENDENT_WATCHDOG);
#endif
}

__attribute__((used)) void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == SYSTICK_TIMER->Instance) {
        HAL_IncTick();
    } else if (htim->Instance == JOINT_SLAVE_SELECT_DEBOUNCE_TIMER->Instance) {
        joint_slave_select_debounce_timer_callback();
    } else if (htim->Instance == JOINT_DELTA_ELAPSED_DEBOUNCE_TIMER->Instance) {
        joint_delta_elapsed_debounce_timer_callback();
    } else if (htim->Instance == WATCHDOG_REFRESH_TIMER->Instance) {
        watchdog_refresh_timer_callback();
    }
}

__attribute__((used)) void HAL_TIM_PWM_PulseFinishedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == DRV8825_PWM_TIMER->Instance) {
        joint_task_pwm_pulse_callback();
    }
}

static inline void joint_slave_select_exti_callback(void)
{
    if (HAL_GPIO_ReadPin(JOINT_SLAVE_SELECT_GPIO, JOINT_SLAVE_SELECT_PIN) ==
        GPIO_PIN_RESET) {
        HAL_TIM_Base_Stop_IT(JOINT_SLAVE_SELECT_DEBOUNCE_TIMER);
        HAL_TIM_Base_Start_IT(JOINT_SLAVE_SELECT_DEBOUNCE_TIMER);
    }
}

static inline void joint_delta_elapsed_exti_callback(void)
{
    if (HAL_GPIO_ReadPin(JOINT_DELTA_ELAPSED_GPIO, JOINT_DELTA_ELAPSED_PIN) ==
        GPIO_PIN_SET) {
        HAL_TIM_Base_Stop_IT(JOINT_DELTA_ELAPSED_DEBOUNCE_TIMER);
        HAL_TIM_Base_Start_IT(JOINT_DELTA_ELAPSED_DEBOUNCE_TIMER);
    }
}

__attribute__((used)) void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == JOINT_SLAVE_SELECT_PIN) {
        // joint_slave_select_exti_callback();
    } else if (GPIO_Pin == JOINT_DELTA_ELAPSED_PIN) {
        joint_delta_elapsed_exti_callback();
    }
}

__attribute__((used)) void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
#ifdef USE_LOG_TASK
    if (huart->Instance == LOG_UART_BUS->Instance) {
        log_task_transmit_done_callback();
    }
#endif
}

__attribute__((used)) void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == PACKET_SPI_BUS->Instance) {
        packet_task_transfer_complete_callback();
    }
}
