#ifndef AS5600_AS5600_REGISTERS_H
#define AS5600_AS5600_REGISTERS_H

#include <stdint.h>

typedef struct {
    uint8_t zmco : 2;
} as5600_zmco_reg_t;

typedef struct {
    uint16_t zpos : 12;
} as5600_zpos_reg_t;

typedef struct {
    uint16_t mpos : 12;
} as5600_mpos_reg_t;

typedef struct {
    uint16_t mang : 12;
} as5600_mang_reg_t;

typedef struct {
    uint8_t wd : 1;
    uint8_t fth : 3;
    uint8_t sf : 2;
    uint8_t pwmf : 2;
    uint8_t outs : 2;
    uint8_t hyst : 2;
    uint8_t pm : 2;
} as5600_conf_reg_t;

typedef struct {
    uint16_t raw_angle : 12;
} as5600_raw_angle_reg_t;

typedef struct {
    uint16_t angle : 12;
} as5600_angle_reg_t;

typedef struct {
    uint8_t md : 1;
    uint8_t ml : 1;
    uint8_t mh : 1;
} as5600_status_reg_t;

typedef struct {
    uint8_t agc : 8;
} as5600_agc_reg_t;

typedef struct {
    uint16_t magnitude : 12;
} as5600_magnitude_reg_t;

#endif // AS5600_AS5600_REGISTERS_H