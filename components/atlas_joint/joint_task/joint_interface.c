#include "joint_manager.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_i2c.h"

static bool frequency_to_prescaler_and_period(uint32_t frequency,
                                              uint32_t clock_hz,
                                              uint32_t clock_div,
                                              uint32_t max_prescaler,
                                              uint32_t max_period,
                                              uint32_t* prescaler,
                                              uint32_t* period)
{
    if (frequency == 0U || !prescaler || !period) {
        return false;
    }

    uint32_t base_clock = clock_hz / (clock_div + 1U);
    uint32_t temp_prescaler = 0U;
    uint32_t temp_period = base_clock / frequency;

    while (temp_period > max_period && temp_prescaler < max_prescaler) {
        temp_prescaler++;
        temp_period = base_clock / ((temp_prescaler + 1U) * frequency);
    }
    if (temp_period > max_period) {
        temp_period = max_period;
        temp_prescaler = (base_clock / (temp_period * frequency)) - 1U;
    }
    if (temp_prescaler > max_prescaler) {
        temp_prescaler = max_prescaler;
    }

    *prescaler = temp_prescaler;
    *period = temp_period;

    return true;
}

drv8825_err_t joint_drv8825_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    joint_config_t* config = (joint_config_t*)user;

    if (config->drv8825_gpio == NULL) {
        return DRV8825_ERR_FAIL;
    }

    HAL_GPIO_WritePin(config->drv8825_gpio, pin, (GPIO_PinState)state);

    return DRV8825_ERR_OK;
}

drv8825_err_t joint_drv8825_pwm_start(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    if (config->drv8825_pwm_timer == NULL) {
        return DRV8825_ERR_FAIL;
    }

    HAL_StatusTypeDef err =
        HAL_TIM_PWM_Start_IT(config->drv8825_pwm_timer, config->drv8825_pwm_channel);

    return err == HAL_OK ? DRV8825_ERR_OK : DRV8825_ERR_FAIL;
}

drv8825_err_t joint_drv8825_pwm_stop(void* user)
{
    joint_config_t* config = (joint_config_t*)user;

    if (config->drv8825_pwm_timer == NULL) {
        return DRV8825_ERR_FAIL;
    }

    HAL_StatusTypeDef err =
        HAL_TIM_PWM_Stop_IT(config->drv8825_pwm_timer, config->drv8825_pwm_channel);

    return err == HAL_OK ? DRV8825_ERR_OK : DRV8825_ERR_FAIL;
}

drv8825_err_t joint_drv8825_pwm_set_frequency(void* user, uint32_t frequency)
{
    joint_config_t* config = (joint_config_t*)user;

    if (config->drv8825_pwm_timer == NULL) {
        return DRV8825_ERR_FAIL;
    }

    uint32_t prescaler;
    uint32_t period;
    if (frequency_to_prescaler_and_period(frequency,
                                          80000000U,
                                          0x0U,
                                          0xFFFFU,
                                          0xFFFFU,
                                          &prescaler,
                                          &period)) {
        __HAL_TIM_DISABLE(config->drv8825_pwm_timer);
        __HAL_TIM_SET_PRESCALER(config->drv8825_pwm_timer, prescaler);
        __HAL_TIM_SET_AUTORELOAD(config->drv8825_pwm_timer, period);
        __HAL_TIM_SET_COMPARE(config->drv8825_pwm_timer, config->drv8825_pwm_channel, period / 2U);
        __HAL_TIM_ENABLE(config->drv8825_pwm_timer);
    }

    return DRV8825_ERR_OK;
}

as5600_err_t joint_as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    joint_config_t* config = (joint_config_t*)user;

    if (config->as5600_gpio == NULL) {
        return AS5600_ERR_FAIL;
    }

    HAL_GPIO_WritePin(config->as5600_gpio, pin, (GPIO_PinState)state);

    return AS5600_ERR_OK;
}

as5600_err_t joint_as5600_bus_write_data(void* user,
                                         uint8_t address,
                                         uint8_t const* data,
                                         size_t data_size)
{
    joint_config_t* config = (joint_config_t*)user;

    HAL_StatusTypeDef err = HAL_I2C_Mem_Write(config->as5600_i2c_bus,
                                              config->as5600_i2c_address << 1,
                                              address,
                                              I2C_MEMADD_SIZE_8BIT,
                                              data,
                                              data_size,
                                              100);

    return err == HAL_OK ? AS5600_ERR_OK : AS5600_ERR_FAIL;
}

as5600_err_t joint_as5600_bus_read_data(void* user,
                                        uint8_t address,
                                        uint8_t* data,
                                        size_t data_size)
{
    joint_config_t* config = (joint_config_t*)user;

    HAL_StatusTypeDef err = HAL_I2C_Mem_Read(config->as5600_i2c_bus,
                                             config->as5600_i2c_address << 1,
                                             address,
                                             I2C_MEMADD_SIZE_8BIT,
                                             data,
                                             data_size,
                                             100);

    return err == HAL_OK ? AS5600_ERR_OK : AS5600_ERR_FAIL;
}

ina226_err_t joint_ina226_bus_write_data(void* user,
                                         uint8_t address,
                                         uint8_t const* data,
                                         size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_config_t* config = (joint_config_t*)user;

    if (config->ina226_i2c_bus == NULL) {
        return INA226_ERR_FAIL;
    }

    HAL_StatusTypeDef err = HAL_I2C_Mem_Write(config->ina226_i2c_bus,
                                              config->ina226_i2c_address,
                                              address,
                                              I2C_MEMADD_SIZE_8BIT,
                                              (uint8_t*)data,
                                              data_size,
                                              100U);

    return err == HAL_OK ? INA226_ERR_OK : INA226_ERR_FAIL;
}

ina226_err_t joint_ina226_bus_read_data(void* user,
                                        uint8_t address,
                                        uint8_t* data,
                                        size_t data_size)
{
    ATLAS_ASSERT(user && data);

    joint_config_t* config = (joint_config_t*)user;

    if (config->ina226_i2c_bus == NULL) {
        return INA226_ERR_FAIL;
    }

    HAL_StatusTypeDef err = HAL_I2C_Mem_Read(config->ina226_i2c_bus,
                                             config->ina226_i2c_address,
                                             address,
                                             I2C_MEMADD_SIZE_8BIT,
                                             data,
                                             data_size,
                                             100U);

    return err == HAL_OK ? INA226_ERR_OK : INA226_ERR_FAIL;
}

step_motor_err_t joint_step_motor_device_set_frequency(void* user, uint32_t frequency)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    drv8825_err_t err = drv8825_set_frequency(&manager->drv8825, frequency);

    return err == DRV8825_ERR_OK ? STEP_MOTOR_ERR_OK : STEP_MOTOR_ERR_FAIL;
}

step_motor_err_t joint_step_motor_device_set_direction(void* user, step_motor_direction_t direction)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    drv8825_err_t err = drv8825_set_direction(&manager->drv8825, (drv8825_direction_t)direction);

    return err == DRV8825_ERR_OK ? STEP_MOTOR_ERR_OK : STEP_MOTOR_ERR_FAIL;
}

motor_driver_err_t joint_motor_driver_motor_set_speed(void* user, float32_t speed)
{
    ATLAS_ASSERT(user);

    joint_manager_t* manager = (joint_manager_t*)user;

    step_motor_err_t err = step_motor_set_speed(&manager->motor, speed);

    return err == STEP_MOTOR_ERR_OK ? MOTOR_DRIVER_ERR_OK : MOTOR_DRIVER_ERR_FAIL;
}

motor_driver_err_t joint_motor_driver_encoder_get_position(void* user, float32_t* position)
{
    ATLAS_ASSERT(user && position);

    joint_manager_t* manager = (joint_manager_t*)user;

    as5600_err_t err = as5600_get_angle_data_scaled_bus(&manager->as5600, position);

    return err == AS5600_ERR_OK ? MOTOR_DRIVER_ERR_OK : MOTOR_DRIVER_ERR_FAIL;
}

motor_driver_err_t joint_motor_driver_regulator_get_control(void* user,
                                                            float32_t error,
                                                            float32_t* control,
                                                            float32_t delta_time)
{
    ATLAS_ASSERT(user && control);

    joint_manager_t* manager = (joint_manager_t*)user;

    *control = pid_regulator_get_sat_control(&manager->regulator, error, delta_time);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t joint_motor_driver_fault_get_current(void* user, float32_t* current)
{
    ATLAS_ASSERT(user && current);

    *current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}
