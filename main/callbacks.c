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

__attribute__((used)) void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == SYSTICK_TIMER->Instance) {
        HAL_IncTick();
    }
#ifdef DELTA_TEST
    else if (htim->Instance == JOINT_DELTA_TIMER->Instance) {
        joint_task_delta_timer_callback();
    }
#endif
#ifdef PACKET_TEST
    else if (htim->Instance == JOINT_CHIP_SELECT_TIMER->Instance) {
        packet_task_joint_packet_ready_callback();
    }
#endif
    else if (htim->Instance == WATCHDOG_TIMER->Instance) {
        if (is_kernel_started) {
            watchdog_task_watchdog_timer_callback();
        } else {
            HAL_IWDG_Refresh(INDEPENDENT_WATCHDOG);
        }
    }
}

__attribute__((used)) void HAL_TIM_PWM_PulseFinishedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == A4988_PWM_TIMER->Instance) {
        joint_task_pwm_pulse_callback();
    }
}

__attribute__((used)) void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == JOINT_CHIP_SELECT_PIN) {
        packet_task_joint_packet_ready_callback();
    } else if (GPIO_Pin == JOINT_DELTA_TIMER_PIN) {
        joint_task_delta_timer_callback();
    }
}

__attribute__((used)) void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
#ifdef LOG_VIA_UART
    if (huart->Instance == LOG_UART_BUS->Instance) {
        log_task_transmit_done_callback();
    }
#endif
}
