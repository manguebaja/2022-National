#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#ifndef MBED_H
    #include "mbed.h"
    #define MBED_H
#endif

#define CAN_IER         (*((volatile unsigned long *)0x40006414))

#define BUFFER_SIZE     50
#define THROTTLE_MID    0x00
#define THROTTLE_RUN    0x01
#define THROTTLE_CHOKE  0x02

#define SYNC_ID         0x001       // message for bus sync
#define THROTTLE_ID     0x100       // 1by = throttle state (0x00, 0x01 or 0x02)
#define FLAGS_ID        0x101       // 1by
#define IMU_ACC_ID      0x200       // 8by = accelerometer data (3D) + timestamp
#define IMU_DPS_ID      0x201       // 8by = gyroscope data (3D) + timestamp 
#define SPEED_ID        0x300       // 4by = speed + timestamp
#define RPM_ID          0x304       // 4by = rpm + timestamp
#define TEMPERATURE_ID  0x400       // 4by = engine temp. + cvt temp. + timestamp
#define FUEL_ID         0x500       // 3by = fuel level + timestamp
#define LAT_ID          0x600       // 1by
#define LNG_ID          0x700       // 1by

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t dps_x;
    int16_t dps_y;
    int16_t dps_z;
} imu_t;
    
typedef struct
{   
    //imu_t imu[4];
    imu_t imu;
    uint16_t rpm;
    uint16_t speed;
    uint8_t temperature;
    uint8_t flags;      // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB
    //float latitude;
    //float longitude;
    uint32_t timestamp;

} packet_t;

#endif