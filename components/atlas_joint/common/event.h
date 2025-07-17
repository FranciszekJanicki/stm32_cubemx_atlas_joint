#ifndef COMMON_EVENT_H
#define COMMON_EVENT_H

#include "atlas_data.h"

typedef enum {
    SYSTEM_EVENT_ORIGIN_PACKET,
    SYSTEM_EVENT_ORIGIN_JOINT,
} system_event_origin_t;

typedef enum {
    SYSTEM_EVENT_TYPE_JOINT_DATA,
    SYSTEM_EVENT_TYPE_START_JOINT,
    SYSTEM_EVENT_TYPE_STOP_JOINT,
} system_event_type_t;

typedef float32_t system_event_payload_joint_data_t;
typedef int system_event_payload_start_joint_t;
typedef int system_event_payload_stop_joint_t;

typedef union {
    system_event_payload_joint_data_t joint_data;
    system_event_payload_start_joint_t start_joint;
    system_event_payload_stop_joint_t stop_joint;
} system_event_payload_t;

typedef struct {
    system_event_origin_t origin;
    system_event_type_t type;
    system_event_payload_t payload;
} system_event_t;

typedef enum {
    JOINT_EVENT_TYPE_START,
    JOINT_EVENT_TYPE_STOP,
    JOINT_EVENT_TYPE_POSITION,
} joint_event_type_t;

typedef int joint_event_payload_start_t;
typedef int joint_event_payload_stop_t;
typedef float32_t joint_event_payload_position_t;

typedef union {
    joint_event_payload_start_t start;
    joint_event_payload_stop_t stop;
    joint_event_payload_position_t position;
} joint_event_payload_t;

typedef struct {
    joint_event_type_t type;
    joint_event_payload_t payload;
} joint_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_JOINT_DATA,
} packet_event_type_t;

typedef int packet_event_payload_start_t;
typedef int packet_event_payload_stop_t;
typedef float32_t packet_event_payload_joint_data_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_joint_data_t joint_data;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H
