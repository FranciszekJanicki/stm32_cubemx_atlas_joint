#include "main.h"
#include "atlas_joint.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();

    atlas_joint_config_t config = {
        .uart_ctx = {.uart = &huart2},
        .packet_ctx = {.config = {.robot_packet_ready_gpio = GPIOA,
                                  .robot_packet_ready_pin = GPIO_PIN_0,
                                  .packet_spi = &hspi1}},
        .joint_ctx = {.config = {.drv8825_pwm_timer = &htim1,
                                 .drv8825_pwm_channel = TIM_CHANNEL_4,
                                 .drv8825_gpio = GPIOA,
                                 .drv8825_dir_pin = GPIO_PIN_10,
                                 .ina226_i2c_bus = &hi2c1,
                                 .ina226_i2c_address = 0x00,
                                 .as5600_i2c_bus = NULL,
                                 .as5600_gpio = GPIOA,
                                 .as5600_dir_pin = 0x00,
                                 .ina226_i2c_address = 0x00},
                      .parameters = {.prop_gain = 10.0F,
                                     .int_gain = 0.0F,
                                     .dot_gain = 0.0F,
                                     .sat_gain = 0.0F,
                                     .min_position = 0.0F,
                                     .max_position = 359.0F,
                                     .min_speed = 10.0F,
                                     .max_speed = 500.0F,
                                     .step_change = 1.8F,
                                     .current_limit = 2.0F}}};

    atlas_joint_initialize(&config);
}
