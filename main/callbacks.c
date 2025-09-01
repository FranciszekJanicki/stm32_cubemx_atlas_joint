#include "FreeRTOS.h"
#include "atlas_joint.h"
#include "common.h"
#include "iwdg.h"
#include "log_task.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "wwdg.h"

__attribute__((used)) void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM4) {
        HAL_IncTick();
    }
#ifdef DELTA_TEST
    else if (htim->Instance == TIM1) {
        joint_task_delta_timer_callback();
    }
#endif
#ifdef PACKET_TEST
    else if (htim->Instance == TIM3) {
        packet_task_joint_packet_ready_callback();
    }
#endif
}

__attribute__((used)) void HAL_TIM_PWM_PulseFinishedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        joint_task_pwm_pulse_callback();
    }
}

__attribute__((used)) void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == 0x0000U) {
        packet_task_joint_packet_ready_callback();
    } else if (GPIO_Pin == 0x0001U) {
        joint_task_delta_timer_callback();
    }
}

__attribute__((used)) void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
#ifdef LOG_VIA_UART
    if (huart->Instance == USART2) {
        log_task_transmit_done_callback();
    }
#endif
}

__attribute__((used)) void HAL_WWDG_EarlyWakeupCallback(
    WWDG_HandleTypeDef* hwwdg)
{
    if (hwwdg->Instance == WWDG) {
        HAL_IWDG_Refresh(&hiwdg);
    }
}