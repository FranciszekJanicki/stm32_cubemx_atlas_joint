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
    SYSTEM_EVENT_TYPE_JOINT_COMMAND,
    SYSTEM_EVENT_TYPE_JOINT_RESPONSE,
} system_event_type_t;

typedef struct {
} system_event_payload_packet_ready_t;

typedef struct {
} system_event_payload_packet_started_t;

typedef struct {
} system_event_payload_packet_stopped_t;

typedef struct {
} system_event_payload_joint_ready_t;

typedef struct {
} system_event_payload_joint_started_t;

typedef struct {
} system_event_payload_joint_stopped_t;

typedef struct {
    atlas_joint_command_t command;
} system_event_payload_joint_command_t;

typedef struct {
    atlas_joint_response_t response;
} system_event_payload_joint_response_t;

typedef union {
    system_event_payload_packet_ready_t packet_ready;
    system_event_payload_packet_started_t packet_started;
    system_event_payload_packet_stopped_t packet_stopped;
    system_event_payload_joint_ready_t joint_ready;
    system_event_payload_joint_started_t joint_started;
    system_event_payload_joint_stopped_t joint_stopped;
    system_event_payload_joint_command_t joint_command;
    system_event_payload_joint_response_t joint_response;
} system_event_payload_t;

typedef struct {
    system_event_type_t type;
    system_event_origin_t origin;
    system_event_payload_t payload;
} system_event_t;

typedef enum {
    JOINT_EVENT_TYPE_START,
    JOINT_EVENT_TYPE_STOP,
    JOINT_EVENT_TYPE_JOINT_COMMAND,
} joint_event_type_t;

typedef struct {
} joint_event_payload_start_t;

typedef struct {
} joint_event_payload_stop_t;

typedef struct {
    atlas_joint_command_t command;
} joint_event_payload_joint_command_t;

typedef union {
    joint_event_payload_start_t start;
    joint_event_payload_stop_t stop;
    joint_event_payload_joint_command_t joint_command;
} joint_event_payload_t;

typedef struct {
    joint_event_type_t type;
    joint_event_payload_t payload;
} joint_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_JOINT_RESPONSE,
} packet_event_type_t;

typedef struct {
} packet_event_payload_start_t;

typedef struct {
} packet_event_payload_stop_t;

typedef struct {
    atlas_joint_response_t response;
} packet_event_payload_joint_response_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_joint_response_t joint_response;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H
