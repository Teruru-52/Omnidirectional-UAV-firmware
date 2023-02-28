/*
 *  mpu9250.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __MPU9250_H_
#define __MPU9250_H_

#include "main.h"

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define GYRO_FACTOR 16.4
#define ACCEL_FACTOR 8192.0

typedef struct
{
    float x;
    float y;
    float z;
} AxesRaw;

uint8_t read_byte(uint8_t reg);
void write_byte(uint8_t reg, uint8_t data);

void Initialize();
void CalcOffset();
void ReadAcc(AxesRaw *acc);
void ReadGyro(AxesRaw *gyro);
void ReadMag(AxesRaw *mag);
void ReadSensor(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag);

#endif /* __MPU9250_H_ */