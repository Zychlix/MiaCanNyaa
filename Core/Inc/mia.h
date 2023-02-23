//
// Created by michal on 18.02.23.
//
#pragma once
#include "stdint.h"

#define CAN_EGV_ACCEL_VAR_ID 0x201
#define CAN_EGV_CMD_VAR_ID 0x301
#define CAN_EGV_SYNC_ALL_ID 0x80
#define CAN_BMS_CHA_ID 0x622
#define MIN_THROTTLE 70

#define MIN_RAW_ADC_ACCEL 30
#define MAX_RAW_ADC_ACCEL 200



typedef struct car
{
    uint8_t in_reverse;

} car_t;

void car_toggle_gear(car_t * car);
