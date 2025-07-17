#ifndef PID_REGULATOR_PID_REGULATOR_CONFIG_H
#define PID_REGULATOR_PID_REGULATOR_CONFIG_H

typedef float float32_t;

typedef struct {
    float32_t prev_error;
    float32_t int_error;
    float32_t dot_error;
    float32_t sat_error;
    float32_t prev_sat_error;
    float32_t int_sat_error;
} pid_regulator_state_t;

typedef struct {
    float32_t prop_gain;
    float32_t int_gain;
    float32_t dot_gain;
    float32_t dot_time;
    float32_t sat_gain;
    float32_t min_control;
    float32_t max_control;
} pid_regulator_config_t;

#endif // PID_REGULATOR_PID_REGULATOR_CONFIG_H