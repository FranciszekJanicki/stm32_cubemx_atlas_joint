#include "FreeRTOS.h"
#include "atlas_joint.h"
#include "common.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "uart_task.h"

__attribute__((used)) void HAL_TIM_PeriodElapsedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM4) {
        HAL_IncTick();
    } else if (htim->Instance == TIM2) {
        #ifdef PACKET_TEST
        joint_task_delta_timer_callback();
    #endif} else if (htim->Instance == TIM3) {
#ifdef PACKET_TEST
        packet_task_joint_packet_ready_callback();
#endif
    }
}

__attribute__((used)) void HAL_TIM_PWM_PulseFinishedCallback(
    TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
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
    if (huart->Instance == USART2) {
        uart_task_transmit_done_callback();
    }
}