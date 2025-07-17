#ifndef ATLAS_CORE_ATLAS_DATA_H
#define ATLAS_CORE_ATLAS_DATA_H

#include "atlas_joint_num.h"
#include <stdbool.h>

#ifndef FLOAT32_T
#define FLOAT32_T
typedef float float32_t;
#endif

#ifndef VEC3_FLOAT32_T
#define VEC3_FLOAT32_T
typedef struct {
    float32_t x;
    float32_t y;
    float32_t z;
} vec3_float32_t;
#endif

typedef struct {
    vec3_float32_t position;
    vec3_float32_t orientation;
} atlas_cartesian_data_t;

typedef struct {
    float32_t positions[ATLAS_JOINT_NUM];
} atlas_joints_data_t;

typedef enum {
    ATLAS_DATA_TYPE_JOINTS,
    ATLAS_DATA_TYPE_CARTESIAN,
    ATLAS_DATA_TYPE_NONE,
} atlas_data_type_t;

typedef union {
    atlas_cartesian_data_t cartesian_data;
    atlas_joints_data_t joints_data;
} atlas_data_payload_t;

typedef struct {
    atlas_data_type_t type;
    atlas_data_payload_t payload;
} atlas_data_t;

bool atlas_is_data_equal(atlas_data_t const* data1, atlas_data_t const* data2);
bool atlas_is_cartesian_data_equal(atlas_cartesian_data_t const* data1,
                                   atlas_cartesian_data_t const* data2);
bool atlas_is_joints_data_equal(atlas_joints_data_t const* data1, atlas_joints_data_t const* data2);

void atlas_print_data(atlas_data_t const* data);
void atlas_print_cartesian_data(atlas_cartesian_data_t const* data);
void atlas_print_joints_data(atlas_joints_data_t const* data);

#endif // ATLAS_CORE_ATLAS_DATA_H
