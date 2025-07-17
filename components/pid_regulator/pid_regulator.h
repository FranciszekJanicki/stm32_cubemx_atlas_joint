#ifndef PID_REGULATOR_PID_REGULATOR_H
#define PID_REGULATOR_PID_REGULATOR_H

#include "pid_regulator_config.h"

typedef struct {
    pid_regulator_state_t state;
    pid_regulator_config_t config;
} pid_regulator_t;

void pid_regulator_initialize(pid_regulator_t* regulator, pid_regulator_config_t const* config);
void pid_regulator_deinitialize(pid_regulator_t* regulator);

void pid_regulator_reset(pid_regulator_t* regulator);

float32_t pid_regulator_get_control(pid_regulator_t* regulator,
                                    float32_t error,
                                    float32_t delta_time);
float32_t pid_regulator_get_sat_control(pid_regulator_t* regulator,
                                        float32_t error,
                                        float32_t delta_time);

#endif // PID_REGULATOR_PID_REGULATOR_H
