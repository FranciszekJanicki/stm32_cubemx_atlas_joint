#ifndef DRV8825_DRV8825_H
#define DRV8825_DRV8825_H

#include "drv8825_config.h"

typedef struct {
    drv8825_config_t config;
    drv8825_interface_t interface;
} drv8825_t;

drv8825_err_t drv8825_initialize(drv8825_t const* drv8825,
                                 drv8825_config_t const* config,
                                 drv8825_interface_t const* interface);
drv8825_err_t drv8825_deinitialize(drv8825_t const* drv8825);

drv8825_err_t drv8825_set_frequency(drv8825_t const* drv8825, uint32_t frequency);

drv8825_err_t drv8825_set_microstep(drv8825_t const* drv8825, drv8825_microstep_t microstep);
drv8825_err_t drv8825_set_full_microstep(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_half_microstep(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_quarter_microstep(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_eighth_microstep(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_sixteenth_microstep(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_thirtysecondth_microstep(drv8825_t const* drv8825);

drv8825_err_t drv8825_set_direction(drv8825_t const* drv8825, drv8825_direction_t direction);
drv8825_err_t drv8825_set_forward_direction(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_backward_direction(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_stop_direction(drv8825_t const* drv8825);

drv8825_err_t drv8825_set_decay(drv8825_t const* drv8825, drv8825_decay_t decay);
drv8825_err_t drv8825_set_fast_decay(drv8825_t const* drv8825);
drv8825_err_t drv8825_set_slow_decay(drv8825_t const* drv8825);

drv8825_err_t drv8825_set_reset(drv8825_t const* drv8825, bool reset);
drv8825_err_t drv8825_set_enable(drv8825_t const* drv8825, bool enable);
drv8825_err_t drv8825_set_sleep(drv8825_t const* drv8825, bool sleep);

#endif // DRV8825_DRV8825_H