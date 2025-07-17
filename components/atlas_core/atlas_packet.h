#ifndef ATLAS_CORE_ATLAS_PACKET_H
#define ATLAS_CORE_ATLAS_PACKET_H

#include "atlas_data.h"
#include "atlas_path.h"
#include "atlas_state.h"

typedef enum {
    ATLAS_ROB_PACKET_TYPE_JOINTS_DATA,
    ATLAS_ROB_PACKET_TYPE_START_JOINTS,
    ATLAS_ROB_PACKET_TYPE_STOP_JOINTS,
} atlas_rob_packet_type_t;

typedef atlas_joints_data_t atlas_rob_packet_payload_joints_data_t;
typedef int atlas_rob_packet_payload_start_joints_t;
typedef int atlas_rob_packet_payload_stop_joints_t;

typedef union {
    atlas_rob_packet_payload_joints_data_t joints_data;
    atlas_rob_packet_payload_start_joints_t start_joints;
    atlas_rob_packet_payload_stop_joints_t stop_joints;
} atlas_rob_packet_payload_t;

typedef struct {
    atlas_rob_packet_type_t type;
    atlas_rob_packet_payload_t payload;
} atlas_rob_packet_t;

typedef enum {
    ATLAS_HMI_PACKET_TYPE_JOINTS_DATA,
} atlas_hmi_packet_type_t;

typedef atlas_joints_data_t atlas_hmi_packet_payload_joints_data_t;

typedef union {
    atlas_hmi_packet_payload_joints_data_t joints_data;
} atlas_hmi_packet_payload_t;

typedef struct {
    atlas_hmi_packet_type_t type;
    atlas_hmi_packet_payload_t payload;
} atlas_hmi_packet_t;

#endif // ATLAS_CORE_ATLAS_PACKET_H