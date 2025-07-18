#ifndef COMMON_EVENT_H
#define COMMON_EVENT_H

#include "atlas_data.h"

typedef enum {
    SYSTEM_EVENT_ORIGIN_PACKET,
    SYSTEM_EVENT_ORIGIN_JOINT,
} system_event_origin_t;

typedef enum {
    SYSTEM_EVENT_TYPE_START,
    SYSTEM_EVENT_TYPE_STOP,
    SYSTEM_EVENT_TYPE_READY,
    SYSTEM_EVENT_TYPE_FAULT,
    SYSTEM_EVENT_TYPE_DATA,
} system_event_type_t;

typedef struct {
} system_event_payload_start_t;
typedef struct {
} system_event_payload_stop_t;
typedef struct {
    float32_t position;
} system_event_payload_data_t;
typedef struct {
} system_event_payload_ready_t;
typedef struct {
} system_event_payload_fault_t;

typedef union {
    system_event_payload_start_t start;
    system_event_payload_stop_t stop;
    system_event_payload_data_t data;
    system_event_payload_fault_t fault;
    system_event_payload_ready_t ready;
} system_event_payload_t;

typedef struct {
    system_event_type_t type;
    system_event_origin_t origin;
    system_event_payload_t payload;
} system_event_t;

typedef enum {
    JOINT_EVENT_TYPE_START,
    JOINT_EVENT_TYPE_STOP,
    JOINT_EVENT_TYPE_DATA,
} joint_event_type_t;

typedef struct {
} joint_event_payload_start_t;
typedef struct {
} joint_event_payload_stop_t;
typedef struct {
    float32_t position;
} joint_event_payload_data_t;

typedef union {
    joint_event_payload_start_t start;
    joint_event_payload_stop_t stop;
    joint_event_payload_data_t data;
} joint_event_payload_t;

typedef struct {
    joint_event_type_t type;
    joint_event_payload_t payload;
} joint_event_t;

typedef enum {
    PACKET_EVENT_TYPE_START,
    PACKET_EVENT_TYPE_STOP,
    PACKET_EVENT_TYPE_DATA,
    PACKET_EVENT_TYPE_FAULT,
    PACKET_EVENT_TYPE_READY,
} packet_event_type_t;

typedef struct {
} packet_event_payload_start_t;
typedef struct {
} packet_event_payload_stop_t;
typedef struct {
    float32_t position;
} packet_event_payload_data_t;
typedef struct {
} packet_event_payload_ready_t;
typedef struct {
} packet_event_payload_fault_t;

typedef union {
    packet_event_payload_start_t start;
    packet_event_payload_stop_t stop;
    packet_event_payload_data_t data;
    packet_event_payload_fault_t fault;
    packet_event_payload_ready_t ready;
} packet_event_payload_t;

typedef struct {
    packet_event_type_t type;
    packet_event_payload_t payload;
} packet_event_t;

#endif // COMMON_EVENT_H
