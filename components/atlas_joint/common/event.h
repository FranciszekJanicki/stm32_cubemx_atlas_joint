#ifndef COMMON_EVENT_H
#define COMMON_EVENT_H

#include "atlas_data.h"

typedef enum {
    SYSTEM_EVENT_ORIGIN_PACKET,
    SYSTEM_EVENT_ORIGIN_JOINT,
} system_event_origin_t;

typedef enum {
    SYSTEM_EVENT_TYPE_PACKET_READY,
    SYSTEM_EVENT_TYPE_PACKET_STARTED,
    SYSTEM_EVENT_TYPE_PACKET_STOPPED,
    SYSTEM_EVENT_TYPE_JOINT_READY,
    SYSTEM_EVENT_TYPE_JOINT_STARTED,
    SYSTEM_EVENT_TYPE_JOINT_STOPPED,
    SYSTEM_EVENT_TYPE_JOINT_FAULT,
    SYSTEM_EVENT_TYPE_JOINT_MEASURE,
    SYSTEM_EVENT_TYPE_JOINT_START,
    SYSTEM_EVENT_TYPE_JOINT_STOP,
    SYSTEM_EVENT_TYPE_JOINT_RESET,
    SYSTEM_EVENT_TYPE_JOINT_REFERENCE,
    SYSTEM_EVENT_TYPE_JOINT_PARAMETERS,
} system_event_type_t;

typedef struct {
} system_event_payload_packet_ready_t;

typedef struct {
} system_event_payload_packet_started_t;

typedef struct {
} system_event_payload_packet_stopped_t;

typedef struct {
    atlas_joint_start_t start;
} system_event_payload_joint_start_t;

typedef struct {
    atlas_joint_stop_t stop;
} system_event_payload_joint_stop_t;

typedef struct {
    atlas_joint_reset_t reset;
} system_event_payload_joint_reset_t;

typedef struct {
    atlas_joint_reference_t reference;
} system_event_payload_joint_reference_t;

typedef struct {
    atlas_joint_parameters_t parameters;
} system_event_payload_joint_parameters_t;

typedef struct {
    atlas_joint_ready_t ready;
} system_event_payload_joint_ready_t;

typedef struct {
    atlas_joint_started_t started;
} system_event_payload_joint_started_t;

typedef struct {
    atlas_joint_stopped_t stopped;
} system_event_payload_joint_stopped_t;

typedef struct {
    atlas_joint_fault_t fault;
} system_event_payload_joint_fault_t;

typedef struct {
    atlas_joint_measure_t measure;
} system_event_payload_joint_measure_t;

typedef union {
    system_event_payload_packet_ready_t packet_ready;
    system_event_payload_packet_started_t packet_started;
    system_event_payload_packet_stopped_t packet_stopped;
    system_event_payload_joint_start_t joint_start;
    system_event_payload_joint_stop_t joint_stop;
    system_event_payload_joint_reset_t joint_reset;
    system_event_payload_joint_reference_t joint_reference;
    system_event_payload_joint_parameters_t joint_parameters;
    system_event_payload_joint_ready_t joint_ready;
    system_event_payload_joint_started_t joint_started;
    system_event_payload_joint_stopped_t joint_stopped;
    system_event_payload_joint_fault_t joint_fault;
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
    JOINT_EVENT_TYPE_RESET,
    JOINT_EVENT_TYPE_REFERENCE,
    JOINT_EVENT_TYPE_PARAMETERS,
} joint_event_type_t;

typedef struct {
    atlas_joint_start_t start;
} joint_event_payload_start_t;

typedef struct {
    atlas_joint_stop_t stop;
} joint_event_payload_stop_t;

typedef struct {
    atlas_joint_reset_t reset;
} joint_event_payload_reset_t;

typedef struct {
    atlas_joint_reference_t reference;
} joint_event_payload_reference_t;

typedef struct {
    atlas_joint_parameters_t parameters;
} joint_event_payload_parameters_t;

typedef union {
    joint_event_payload_start_t start;
    joint_event_payload_stop_t stop;
    joint_event_payload_reset_t reset;
    joint_event_payload_reference_t reference;
    joint_event_payload_parameters_t parameters;
} joint_event_payload_t;

typedef struct {
    joint_event_type_t type;
    joint_event_payload_t payload;
} joint_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_JOINT_READY,
    PACKET_EVENT_TYPE_JOINT_STARTED,
    PACKET_EVENT_TYPE_JOINT_STOPPED,
    PACKET_EVENT_TYPE_JOINT_FAULT,
    PACKET_EVENT_TYPE_JOINT_MEASURE,
} packet_event_type_t;

typedef struct {
} packet_event_payload_start_t;

typedef struct {
} packet_event_payload_stop_t;

typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_ready_t ready;
} packet_event_payload_joint_ready_t;

typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_started_t started;
} packet_event_payload_joint_started_t;

typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_stopped_t stopped;
} packet_event_payload_joint_stopped_t;

typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_fault_t fault;
} packet_event_payload_joint_fault_t;

typedef struct {
    atlas_joint_num_t num;
    atlas_timestamp_t timestamp;
    atlas_joint_measure_t measure;
} packet_event_payload_joint_measure_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_joint_ready_t joint_ready;
    packet_event_payload_joint_started_t joint_started;
    packet_event_payload_joint_stopped_t joint_stopped;
    packet_event_payload_joint_fault_t joint_fault;
    packet_event_payload_joint_measure_t joint_measure;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H
