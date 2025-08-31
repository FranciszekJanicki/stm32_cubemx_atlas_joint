#include "main.h"
#include "atlas_joint.h"
#include "config.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

static atlas_joint_config_t config = {
    .log_ctx =
        {
#ifdef LOG_VIA_UART
            .log_bus = LOG_UART_BUS
#else
            .log_bus = LOG_USB_BUS
#endif
        },
    .system_ctx = {.config = {.num = JOINT_NUM,
                              .timestamp_rtc = TIMESTAMP_RTC,
#ifdef DELTA_TEST
                              .delta_timer = DELTA_TIMER,
#endif
                              .delta_time_elapsed_gpio =
                                  DELTA_TIME_ELAPSED_GPIO,
                              .delta_time_elapsed_pin =
                                  DELTA_TIME_ELAPSED_PIN}},
    .packet_ctx =
        {.config = {.robot_packet_ready_gpio = ROBOT_PACKET_READY_GPIO,
                    .robot_packet_ready_pin = ROBOT_PACKET_READY_PIN,
                    .joint_packet_ready_gpio = JOINT_PACKET_READY_GPIO,
                    .joint_packet_ready_pin = JOINT_PACKET_READY_PIN,
                    .packet_spi_bus = PACKET_SPI_BUS,
#ifdef PACKET_TEST
                    .joint_packet_ready_timer = JOINT_PACKET_READY_TIMER
#endif
         }},
    .joint_ctx = {.config = {.a4988_pwm_timer = A4988_PWM_TIMER,
                             .a4988_pwm_channel = A4988_PWM_CHANNEL,
                             .a4988_dir_gpio = A4988_DIR_GPIO,
                             .a4988_dir_pin = A4988_DIR_PIN,
                             .ina226_i2c_bus = INA226_I2C_BUS,
                             .ina226_i2c_address = INA226_I2C_ADDRESS,
                             .as5600_i2c_bus = AS5600_I2C_BUS,
                             .as5600_i2c_address = AS5600_I2C_ADDRESS,
                             .as5600_dir_gpio = AS5600_DIR_GPIO,
                             .as5600_dir_pin = AS5600_DIR_PIN},
                  .parameters = {.prop_gain = JOINT_PROP_GAIN,
                                 .int_gain = JOINT_INT_GAIN,
                                 .dot_gain = JOINT_DOT_GAIN,
                                 .sat_gain = JOINT_SAT_GAIN,
                                 .dead_error = JOINT_DEAD_ERROR,
                                 .min_position = JOINT_MIN_POSITION,
                                 .max_position = JOINT_MAX_POSITION,
                                 .min_speed = JOINT_MIN_SPEED,
                                 .max_speed = JOINT_MAX_SPEED,
                                 .min_acceleration = JOINT_MIN_ACCELERATION,
                                 .max_acceleration = JOINT_MAX_ACCELERATION,
                                 .step_change = JOINT_STEP_CHANGE,
                                 .current_limit = JOINT_CURRENT_LIMIT,
                                 .magnet_polarity = JOINT_MAGNET_POLARITY}}};

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    // #ifdef LOG_VIA_UART
    //     MX_USART2_UART_Init();
    // #else
    //     MX_USB_DEVICE_Init();
    // #endif
    // MX_TIM1_Init();
    MX_TIM2_Init();
    // MX_TIM3_Init();
    // MX_I2C1_Init();
    // MX_SPI1_Init();
    // MX_RTC_Init();

    HAL_Delay(500);

    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // __HAL_TIM_SET_AUTORELOAD(&htim2, 10000U);
    // __HAL_TIM_SET_PRESCALER(&htim2, 0U);
    // __HAL_TIM_SET_COUNTER(&htim2, 0U);
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10000U);

    HAL_GPIO_WritePin(GPIOA, 1 << 0, 0);

    // atlas_joint_initialize(&config);
}
