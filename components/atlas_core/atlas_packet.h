#ifndef ATLAS_CORE_ATLAS_PACKET_H
#define ATLAS_CORE_ATLAS_PACKET_H

#include "atlas_data.h"
#include "atlas_path.h"
#include "atlas_state.h"

typedef enum {
    ATLAS_ROBOT_PACKET_TYPE_READY,
    ATLAS_ROBOT_PACKET_TYPE_FAULT,
    ATLAS_ROBOT_PACKET_TYPE_DATA,
} atlas_robot_packet_type_t;

typedef enum {
    ATLAS_ROBOT_PACKET_ORIGIN_1 = ATLAS_JOINT_NUM_1,
    ATLAS_ROBOT_PACKET_ORIGIN_2 = ATLAS_JOINT_NUM_2,
    ATLAS_ROBOT_PACKET_ORIGIN_3 = ATLAS_JOINT_NUM_3,
    ATLAS_ROBOT_PACKET_ORIGIN_4 = ATLAS_JOINT_NUM_4,
    ATLAS_ROBOT_PACKET_ORIGIN_5 = ATLAS_JOINT_NUM_5,
    ATLAS_ROBOT_PACKET_ORIGIN_6 = ATLAS_JOINT_NUM_6,
} atlas_robot_packet_origin_t;

typedef struct {
} atlas_robot_packet_payload_ready_t;
typedef struct {
} atlas_robot_packet_payload_fault_t;
typedef struct {
    float32_t position;
} atlas_robot_packet_payload_data_t;

typedef union {
    atlas_robot_packet_payload_ready_t ready;
    atlas_robot_packet_payload_fault_t fault;
    atlas_robot_packet_payload_data_t data;
} atlas_robot_packet_payload_t;

typedef struct {
    atlas_robot_packet_type_t type;
    atlas_robot_packet_origin_t origin;
    atlas_robot_packet_payload_t payload;
} atlas_robot_packet_t;

typedef enum {
    ATLAS_JOINT_PACKET_TYPE_START,
    ATLAS_JOINT_PACKET_TYPE_STOP,
    ATLAS_JOINT_PACKET_TYPE_DATA,
} atlas_joint_packet_type_t;

typedef struct {
} atlas_joint_packet_payload_start_t;
typedef struct {
} atlas_joint_packet_payload_stop_t;
typedef struct {
    float32_t position;
} atlas_joint_packet_payload_data_t;

typedef union {
    atlas_joint_packet_payload_start_t start;
    atlas_joint_packet_payload_stop_t stop;
    atlas_joint_packet_payload_data_t data;
} atlas_joint_packet_payload_t;

typedef struct {
    atlas_joint_packet_type_t type;
    atlas_joint_packet_payload_t payload;
} atlas_joint_packet_t;

#endif // ATLAS_CORE_ATLAS_PACKET_H
