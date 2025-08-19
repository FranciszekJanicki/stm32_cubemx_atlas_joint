#ifndef MAIN_JOINT_CONFIG_H
#define MAIN_JOINT_CONFIG_H

#define JOINT_NUM (ATLAS_JOINT_NUM_1)

#define JOINT_PARAMS         \
    {.prop_gain = 10.0F,     \
     .int_gain = 0.0F,       \
     .dot_gain = 0.0F,       \
     .sat_gain = 0.0F,       \
     .min_position = 0.0F,   \
     .max_position = 359.0F, \
     .min_speed = 0.5F,      \
     .max_speed = 500.0F,    \
     .step_change = 1.8F,    \
     .current_limit = 2.0F,  \
     .dead_error = 1.8F}

#endif // MAIN_JOINT_CONFIG_H