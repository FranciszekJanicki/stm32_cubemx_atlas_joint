#include "atlas_path.h"
#include "atlas_data.h"
#include "atlas_utility.h"

bool atlas_is_path_equal(atlas_path_t const* path1, atlas_path_t const* path2)
{
    ATLAS_ASSERT(path1 && path2);

    if (path1->type != path2->type) {
        return false;
    }

    if (path1->type == ATLAS_PATH_TYPE_JOINTS) {
        return atlas_is_joints_path_equal(&path1->payload.joints_path, &path2->payload.joints_path);
    } else if (path1->type == ATLAS_PATH_TYPE_CARTESIAN) {
        return atlas_is_cartesian_path_equal(&path1->payload.cartesian_path,
                                             &path2->payload.cartesian_path);
    }

    return false;
}

bool atlas_is_cartesian_path_equal(atlas_cartesian_path_t const* path1,
                                   atlas_cartesian_path_t const* path2)
{
    ATLAS_ASSERT(path1 && path2);

    for (uint8_t num = 0U; num < ATLAS_CARTESIAN_PATH_MAX_POINTS; ++num) {
        if (!atlas_is_cartesian_data_equal(&path1->points[num], &path2->points[num])) {
            return false;
        }
    }

    return true;
}

bool atlas_is_joints_path_equal(atlas_joints_path_t const* path1, atlas_joints_path_t const* path2)
{
    ATLAS_ASSERT(path1 && path2);

    for (uint8_t num = 0U; num < ATLAS_JOINTS_PATH_MAX_POINTS; ++num) {
        if (!atlas_is_joints_data_equal(&path1->points[num], &path2->points[num])) {
            return false;
        }
    }

    return true;
}

void atlas_print_path(atlas_path_t const* path)
{
    ATLAS_ASSERT(path);

    if (path->type == ATLAS_PATH_TYPE_CARTESIAN) {
        atlas_print_cartesian_path(&path->payload.cartesian_path);
    } else if (path->type == ATLAS_PATH_TYPE_JOINTS) {
        atlas_print_joints_path(&path->payload.joints_path);
    }
}

void atlas_print_cartesian_path(atlas_cartesian_path_t const* path)
{
    ATLAS_ASSERT(path);

    for (uint8_t point = 0U; point < ATLAS_CARTESIAN_PATH_MAX_POINTS; ++point) {
        atlas_print_cartesian_data(&path->points[point]);
    }
}

void atlas_print_joints_path(atlas_joints_path_t const* path)
{
    ATLAS_ASSERT(path);

    for (uint8_t point = 0U; point < ATLAS_JOINTS_PATH_MAX_POINTS; ++point) {
        atlas_print_joints_data(&path->points[point]);
    }
}
