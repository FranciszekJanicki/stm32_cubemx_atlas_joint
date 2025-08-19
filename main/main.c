#include "main.h"
#include "atlas_joint.h"
#include "gpio.h"
#include "i2c.h"
#include "joint_config.h"
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

static atlas_joint_config_t config = {
    .uart_ctx = {.uart_bus = &huart2},
    .system_ctx = {.config = {.num = JOINT_NUM,
                              .timestamp_rtc = &hrtc,
                              .packet_ready_timer = &htim3,
                              .delta_timer = &htim1}},
    .packet_ctx = {.config = {.robot_packet_ready_gpio = GPIOA,
                              .robot_packet_ready_pin = GPIO_PIN_0,
                              .packet_spi = &hspi1}},
    .joint_ctx = {.config = {.a4988_pwm_timer = &htim2,
                             .a4988_pwm_channel = TIM_CHANNEL_1,
                             .a4988_gpio = GPIOA,
                             .a4988_dir_pin = GPIO_PIN_9,
                             .ina226_i2c_bus = &hi2c1,
                             .ina226_i2c_address = 0x00,
                             .as5600_i2c_bus = &hi2c1,
                             .as5600_gpio = GPIOC,
                             .as5600_i2c_address = AS5600_SLAVE_ADDRESS,
                             .as5600_dir_pin = GPIO_PIN_2},
                  .parameters = JOINT_PARAMS}};

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_RTC_Init();

    HAL_Delay(50);

    atlas_joint_initialize(&config);
}
