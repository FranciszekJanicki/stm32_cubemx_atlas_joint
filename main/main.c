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

static atlas_joint_config_t config = {
    .uart_ctx = {.uart_bus = LOG_UART_BUS},
    .system_ctx = {.config = {.num = JOINT_NUM,
                              .timestamp_rtc = TIMESTAMP_RTC,
                              .packet_ready_timer = PACKET_READY_TIMER,
                              .delta_timer = DELTA_TIMER}},
    .packet_ctx = {.config = {.robot_packet_ready_gpio = PACKET_READY_GPIO,
                              .robot_packet_ready_pin = PACKET_READY_PIN,
                              .packet_spi = PACKET_SPI_BUS}},
    .joint_ctx = {.config = {.a4988_pwm_timer = A4988_PWM_TIMER,
                             .a4988_pwm_channel = A4988_PWM_CHANNEL,
                             .a4988_gpio = A4988_DIR_GPIO,
                             .a4988_dir_pin = A4988_DIR_PIN,
                             .ina226_i2c_bus = INA226_I2C_BUS,
                             .ina226_i2c_address = INA226_I2C_ADDRESS,
                             .as5600_i2c_bus = AS5600_I2C_BUS,
                             .as5600_i2c_address = AS5600_SLAVE_ADDRESS,
                             .as5600_gpio = AS5600_DIR_GPIO,
                             .as5600_dir_pin = AS5600_DIR_PIN},
                  .parameters = {.prop_gain = JOINT_PROP_GAIN,
                                 .int_gain = JOINT_INT_GAIN,
                                 .dot_gain = JOINT_DOT_GAIN,
                                 .sat_gain = JOINT_SAT_GAIN,
                                 .dead_error = JOINT_DEAD_ERROR,
                                 .min_speed = JOINT_MIN_SPEED,
                                 .max_speed = JOINT_MAX_SPEED,
                                 .min_position = JOINT_MIN_POSITION,
                                 .max_position = JOINT_MAX_POSITION,
                                 .step_change = JOINT_STEP_CHANGE,
                                 .current_limit = JOINT_CURRENT_LIMIT}}};

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
