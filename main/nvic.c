#include "FreeRTOS.h"
#include "common.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"

extern void joints_delta_timer_callback(void);
extern void rob_packet_ready_callback(void);
extern void joint_pwm_pulse_callback(atlas_joint_num_t num);
extern void uart_tx_complete_callback(void);
extern void safety_relay_trig_callback(void);

__attribute__((used)) void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM4) {
        HAL_IncTick();
    } else if (htim->Instance == TIM2) {
        joints_delta_timer_callback();
    } else if (htim->Instance == TIM3) {
        rob_packet_ready_callback();
    }
}

__attribute__((used)) void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        joint_pwm_pulse_callback(ATLAS_JOINT_NUM_1);
    }
}

__attribute__((used)) void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == 0x0000U) {
        rob_packet_ready_callback();
    } else if (GPIO_Pin == 0x0001) {
        safety_relay_trig_callback();
    }
}

__attribute__((used)) void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2) {
        uart_tx_complete_callback();
    }
}