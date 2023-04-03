/*
 *  mpu9250.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __MPU9250_H_
#define __MPU9250_H_

#include "main.h"

// MPU9250 registers
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define I2C_MST_CTRL 0x24
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_REG 0x26
#define I2C_SLV0_CTRL 0x27
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
#define EXT_SENS_DATA_00 0x49
#define I2C_SLV0_DO 0x63
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define WHO_AM_I 0x75

#define GYRO_FACTOR 16.4
#define ACCEL_FACTOR -8192.0

// AK8963 registers
#define AK8963_WHO_AM_I 0x00
#define AK8963_HXL 0x03
#define AK8963_HXH 0x04
#define AK8963_HYL 0x05
#define AK8963_HYH 0x06
#define AK8963_HZL 0x07
#define AK8963_HZH 0x08
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_I2C_ADDR 0x0C // phisical address
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

#define max_cali_count 1000

typedef struct
{
    float x;
    float y;
    float z;
} AxesRaw;

void CheckWhoAmI_MPU9250();
void CheckWhoAmI_AK8963();
void InitializeIMU();
void CalcAccOffset(AxesRaw *acc);
void CalcGyroOffset(AxesRaw *gyro);
void SetInitialMag(AxesRaw *mag);
void ReadRawAcc(AxesRaw *acc);
void ReadRawGyro(AxesRaw *gyro);
void ReadRawMag(AxesRaw *mag);
void ReadSensor(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag);

extern const float g;
extern float mn;
extern float me;
extern float md;

#endif /* __MPU9250_H_ */