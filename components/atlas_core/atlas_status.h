#ifndef ATLAS_CORE_ATLAS_STATUS_H
#define ATLAS_CORE_ATLAS_STATUS_H

#include <stdint.h>

typedef struct {
    atlas_joints_data_t data;
    atlas_joints_path_t path;
    atlas_config_t config;
    uint32_t path_index;
} atlas_status_t;

#endif // ATLAS_CORE_ATLAS_STATUS_H