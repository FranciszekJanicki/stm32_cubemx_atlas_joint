#ifndef COMMON_EVENT_H
#define COMMON_EVENT_H

#include "atlas_data.h"

typedef enum {
    SYSTEM_EVENT_ORIGIN_PACKET,
    SYSTEM_EVENT_ORIGIN_JOINT,
} system_event_origin_t;

typedef enum {
    SYSTEM_EVENT_TYPE_JOINT_START,
    SYSTEM_EVENT_TYPE_JOINT_STOP,
    SYSTEM_EVENT_TYPE_JOINT_REFERENCE,
    SYSTEM_EVENT_TYPE_JOINT_MEASURE,
} system_event_type_t;

typedef atlas_joint_start_t system_event_payload_joint_start_t;
typedef atlas_joint_stop_t system_event_payload_joint_stop_t;
typedef atlas_joint_reference_t system_event_payload_joint_reference_t;
typedef atlas_joint_measure_t system_event_payload_joint_measure_t;

typedef union {
    system_event_payload_joint_start_t joint_start;
    system_event_payload_joint_stop_t joint_stop;
    system_event_payload_joint_reference_t joint_reference;
    system_event_payload_joint_measure_t joint_measure;
} system_event_payload_t;

typedef struct {
    system_event_type_t type;
    system_event_origin_t origin;
    system_event_payload_t payload;
} system_event_t;

typedef enum {
    JOINT_EVENT_TYPE_START,
    JOINT_EVENT_TYPE_STOP,
    JOINT_EVENT_TYPE_REFERENCE,
} joint_event_type_t;

typedef atlas_joint_start_t joint_event_payload_start_t;
typedef atlas_joint_stop_t joint_event_payload_stop_t;
typedef atlas_joint_reference_t joint_event_payload_reference_t;

typedef union {
    joint_event_payload_start_t start;
    joint_event_payload_stop_t stop;
    joint_event_payload_reference_t reference;
} joint_event_payload_t;

typedef struct {
    joint_event_type_t type;
    joint_event_payload_t payload;
} joint_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_JOINT_MEASURE,
    PACKET_EVENT_TYPE_JOINT_FAULT,
    PACKET_EVENT_TYPE_JOINT_READY,
} packet_event_type_t;

typedef int packet_event_payload_start_t;
typedef int packet_event_payload_stop_t;
typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_measure_t measure;
} packet_event_payload_joint_measure_t;
typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_ready_t ready;
} packet_event_payload_joint_ready_t;
typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_fault_t fault;
} packet_event_payload_joint_fault_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_joint_measure_t joint_measure;
    packet_event_payload_joint_fault_t joint_fault;
    packet_event_payload_joint_ready_t joint_ready;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H
