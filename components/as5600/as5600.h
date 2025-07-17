#ifndef AS5600_AS5600_H
#define AS5600_AS5600_H

#include "as5600_config.h"
#include "as5600_registers.h"

typedef struct {
    as5600_config_t config;
    as5600_interface_t interface;
} as5600_t;

as5600_err_t as5600_initialize(as5600_t* as5600,
                               as5600_config_t const* config,
                               as5600_interface_t const* interface);
as5600_err_t as5600_deinitialize(as5600_t* as5600);

as5600_err_t as5600_set_direction(as5600_t const* as5600, as5600_direction_t direction);

as5600_err_t as5600_get_angle_data_scaled_adc(as5600_t const* as5600, float32_t* scaled);
as5600_err_t as5600_get_angle_data_raw_adc(as5600_t const* as5600, uint16_t* raw);

as5600_err_t as5600_get_angle_data_scaled_bus(as5600_t const* as5600, float32_t* scaled);
as5600_err_t as5600_get_angle_data_raw_bus(as5600_t const* as5600, uint16_t* raw);

as5600_err_t as5600_get_zmco_reg(as5600_t const* as5600, as5600_zmco_reg_t* reg);

as5600_err_t as5600_get_zpos_reg(as5600_t const* as5600, as5600_zpos_reg_t* reg);
as5600_err_t as5600_set_zpos_reg(as5600_t const* as5600, as5600_zpos_reg_t const* reg);

as5600_err_t as5600_get_mpos_reg(as5600_t const* as5600, as5600_mpos_reg_t* reg);
as5600_err_t as5600_set_mpos_reg(as5600_t const* as5600, as5600_mpos_reg_t const* reg);

as5600_err_t as5600_get_mang_reg(as5600_t const* as5600, as5600_mang_reg_t* reg);
as5600_err_t as5600_set_mang_reg(as5600_t const* as5600, as5600_mang_reg_t const* reg);

as5600_err_t as5600_get_conf_reg(as5600_t const* as5600, as5600_conf_reg_t* reg);
as5600_err_t as5600_set_conf_reg(as5600_t const* as5600, as5600_conf_reg_t const* reg);

as5600_err_t as5600_get_raw_angle_reg(as5600_t const* as5600, as5600_raw_angle_reg_t* reg);

as5600_err_t as5600_get_angle_reg(as5600_t const* as5600, as5600_angle_reg_t* reg);

as5600_err_t as5600_get_status_reg(as5600_t const* as5600, as5600_status_reg_t* reg);

as5600_err_t as5600_get_agc_reg(as5600_t const* as5600, as5600_agc_reg_t* reg);

as5600_err_t as5600_get_magnitude_reg(as5600_t const* as5600, as5600_magnitude_reg_t* reg);

as5600_err_t as5600_send_burn_angle_cmd(as5600_t const* as5600);

as5600_err_t as5600_send_burn_setting_cmd(as5600_t const* as5600);

#endif // AS5600_AS5600_H