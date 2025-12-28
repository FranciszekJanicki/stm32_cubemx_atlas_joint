#include "main.h"
#include "atlas_joint.h"
#include "config.h"
#include "crc.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "wwdg.h"

static atlas_joint_config_t config = {
    .log_ctx = {},
    .system_ctx = {.config = {.delta_timer_gpio = JOINT_DELTA_GPIO,
                              .delta_timer_pin = JOINT_DELTA_PIN}},
    .packet_ctx = {.config = {.joint_num = JOINT_NUM,
                              .data_ready_gpio = JOINT_DRDY_GPIO,
                              .data_ready_pin = JOINT_DRDY_PIN,
                              .slave_select_gpio = JOINT_NSS_GPIO,
                              .slave_select_pin = JOINT_NSS_PIN,
                              .packet_spi_bus = JOINT_SPI_BUS,
                              .timestamp_rtc = TIMESTAMP_RTC}},
    .joint_ctx = {.config = {.drv8825_pwm_timer = DRV8825_STEP_TIMER,
                             .drv8825_pwm_channel = DRV8825_STEP_CHANNEL,
                             .drv8825_dir_gpio = DRV8825_DIR_GPIO,
                             .drv8825_dir_pin = DRV8825_DIR_PIN,
                             .drv8825_en_gpio = DRV8825_EN_GPIO,
                             .drv8825_en_pin = DRV8825_EN_PIN,
                             .drv8825_m0_gpio = DRV8825_M0_GPIO,
                             .drv8825_m0_pin = DRV8825_M0_PIN,
                             .drv8825_m1_gpio = DRV8825_M1_GPIO,
                             .drv8825_m1_pin = DRV8825_M1_PIN,
                             .drv8825_m2_gpio = DRV8825_M2_GPIO,
                             .drv8825_m2_pin = DRV8825_M2_PIN,
                             .ina226_i2c_bus = INA226_I2C_BUS,
                             .ina226_i2c_address = INA226_I2C_ADDRESS,
                             .as5600_i2c_bus = AS5600_I2C_BUS,
                             .as5600_i2c_address = AS5600_I2C_ADDRESS,
                             .as5600_dir_gpio = AS5600_DIR_GPIO,
                             .as5600_dir_pin = AS5600_DIR_PIN},
                  .parameters = {.prop_gain = DEFAULT_PROP_GAIN,
                                 .int_gain = DEFAULT_INT_GAIN,
                                 .dot_gain = DEFAULT_DOT_GAIN,
                                 .sat_gain = DEFAULT_SAT_GAIN,
                                 .dead_error = DEFAULT_DEAD_ERROR,
                                 .min_position = DEFAULT_MIN_POSITION,
                                 .max_position = DEFAULT_MAX_POSITION,
                                 .min_speed = DEFAULT_MIN_SPEED,
                                 .max_speed = DEFAULT_MAX_SPEED,
                                 .min_acceleration = DEFAULT_MIN_ACCELERATION,
                                 .max_acceleration = DEFAULT_MAX_ACCELERATION,
                                 .step_change = DEFAULT_STEP_CHANGE,
                                 .microstep = DEFAULT_MICROSTEP,
                                 .current_limit = DEFAULT_CURRENT_LIMIT,
                                 .magnet_polarity = DEFAULT_MAGNET_POLARITY}}};

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_RTC_Init();
    MX_CRC_Init();
#ifdef USE_WATCHDOG
    MX_TIM5_Init();
    MX_IWDG_Init();
    MX_WWDG_Init();
#endif

    HAL_Delay(1000U);

    atlas_joint_initialize(&config);
}
