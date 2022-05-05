#ifndef FRONTDEFS_H
#define FRONTDEFS_H

/* Conversion definitions */
#define WHEEL_DIAMETER      0.5842      // m
#define PI                  3.1416
#define RAD_TO_DEGREE       180.0/PI
#define TO_G                2.0/32768.0
#define TO_DPS              245.0/32768.0
#define WHEEL_HOLES_NUMBER_MB1  24
#define WHEEL_HOLES_NUMBER_MB2  24
#define HORN_PERIOD         2
#define DEBOUNCE_TIME       0.1         // s
#define IMU_TRIES           10

typedef enum
{
    IDLE_ST,        // wait
    SLOWACQ_ST,     // acquire battery level data
    IMU_ST,         // acquire accelerometer and gyroscope data
    SPEED_ST,       // calculate speed
    THROTTLE_ST,    // check throttle position (from user button)
    DISPLAY_ST,     // send data for display over serial port
    DEBUG_ST        // send data for debug
} state_t;

#endif

