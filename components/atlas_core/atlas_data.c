#include "atlas_data.h"
#include "atlas_joint_num.h"
#include "atlas_log.h"
#include "atlas_utility.h"
#include <stdint.h>

bool atlas_is_data_equal(atlas_data_t const* data1, atlas_data_t const* data2)
{
    ATLAS_ASSERT(data1 && data2);

    if (data1->type != data2->type) {
        return false;
    }

    if (data1->type == ATLAS_DATA_TYPE_JOINTS) {
        return atlas_is_joints_data_equal(&data1->payload.joints_data, &data2->payload.joints_data);
    } else if (data1->type == ATLAS_DATA_TYPE_CARTESIAN) {
        return atlas_is_cartesian_data_equal(&data1->payload.cartesian_data,
                                             &data2->payload.cartesian_data);
    }

    return false;
}

bool atlas_is_cartesian_data_equal(atlas_cartesian_data_t const* data1,
                                   atlas_cartesian_data_t const* data2)
{
    ATLAS_ASSERT(data1 && data2);

    return data1->position.x == data2->position.x && data1->position.y == data2->position.y &&
           data1->position.z == data2->position.z && data1->orientation.x == data2->orientation.x &&
           data1->orientation.y == data2->orientation.y &&
           data1->orientation.z == data2->orientation.z;
}

bool atlas_is_joints_data_equal(atlas_joints_data_t const* data1, atlas_joints_data_t const* data2)
{
    ATLAS_ASSERT(data1 && data2);

    for (uint8_t num = 0; num < ATLAS_JOINT_NUM; ++num) {
        if (data1->positions[num] != data2->positions[num]) {
            return false;
        }
    }

    return true;
}

void atlas_print_data(atlas_data_t const* data)
{
    ATLAS_ASSERT(data);

    if (data->type == ATLAS_DATA_TYPE_JOINTS) {
        atlas_print_joints_data(&data->payload.joints_data);
    } else if (data->type == ATLAS_DATA_TYPE_CARTESIAN) {
        atlas_print_cartesian_data(&data->payload.cartesian_data);
    }
}

void atlas_print_cartesian_data(atlas_cartesian_data_t const* data)
{
    ATLAS_ASSERT(data);

    atlas_log("position x: %d, y: %d, z: %d orientation x: %d, y: %d, z: %d\n\r",
              (int32_t)data->position.x * 100,
              (int32_t)data->position.y * 100,
              (int32_t)data->position.z * 100,
              (int32_t)data->orientation.x * 100,
              (int32_t)data->orientation.y * 100,
              (int32_t)data->orientation.z * 100);
}

void atlas_print_joints_data(atlas_joints_data_t const* data)
{
    ATLAS_ASSERT(data);

    atlas_log("position 1: %d, 2: %d, 3: %d, 4: %d, 5: %d, 6: %d\n\r",
              (int32_t)data->positions[ATLAS_JOINT_NUM_1] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_2] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_3] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_4] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_5] * 100,
              (int32_t)data->positions[ATLAS_JOINT_NUM_6] * 100);
}
