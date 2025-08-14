#ifndef JOINT_TASK_JOINT_TASK_H
#define JOINT_TASK_JOINT_TASK_H

#include "common.h"
#include "joint_manager.h"

typedef struct {
    joint_config_t config;
    joint_parameters_t parameters;
} joint_task_ctx_t;

atlas_err_t joint_task_initialize(joint_task_ctx_t* task_ctx);

void joint_task_delta_timer_callback(void);
void joint_task_pwm_pulse_callback(void);

#endif // JOINT_TASK_JOINT_TASK_H