/*
 *  mpu9250.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "mpu9250.h"

float ax_offset = 0, ay_offset = 0, az_offset = 0;

uint8_t read_byte(uint8_t reg)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00; // dummy

    Write_GPIO(SPI1_CS, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI1_CS, GPIO_PIN_SET);

    return rx_data[1];
}

void write_byte(uint8_t reg, uint8_t data)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg & 0x7F;
    //   tx_data[0] = reg | 0x00;
    tx_data[1] = data; // write data

    Write_GPIO(SPI1_CS, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI1_CS, GPIO_PIN_SET);
}

void Initialize()
{
    uint8_t who_am_i;
    Write_GPIO(SPI1_CS, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(&hspi1); // clockが動かないように、あらかじめEnableにしておく

    HAL_Delay(100);                          // wait start up
    who_am_i = read_byte(WHO_AM_I);          // read who am i
    printf("who_am_i = 0x%x\r\n", who_am_i); // check who am i value
    HAL_Delay(10);
    while (who_am_i != 0x70)
    {
        who_am_i = read_byte(WHO_AM_I);
        printf("who_am_i = 0x%x\r\n", who_am_i);
        HAL_Delay(20);
    }

    HAL_Delay(50);
    write_byte(PWR_MGMT_1, 0x00); // set pwr_might (20MHz)
    HAL_Delay(50);
    write_byte(CONFIG, 0x00); // set config (FSYNCはNC)
    HAL_Delay(50);
    write_byte(GYRO_CONFIG, 0x18); // set gyro config (2000dps)
    HAL_Delay(50);
    write_byte(ACCEL_CONFIG, 0x08); // set acc config (4g)
    HAL_Delay(50);
    // write_byte(0x1D, 0x00); // LPF (Accelerometer, Bandwidth460 Hz)
    // HAL_Delay(50);
}

void ReadAcc(AxesRaw *acc)
{
    int16_t acc_raw;
    // H:8bit shift, Link h and l
    acc_raw = (int16_t)((int16_t)(read_byte(ACCEL_XOUT_H) << 8) | read_byte(ACCEL_XOUT_L));
    acc->x = (float)(acc_raw) / ACCEL_FACTOR - ax_offset;
    acc_raw = (int16_t)((int16_t)(read_byte(ACCEL_YOUT_H) << 8) | read_byte(ACCEL_YOUT_L));
    acc->y = (float)(acc_raw) / ACCEL_FACTOR - ay_offset;
    acc_raw = (int16_t)((int16_t)(read_byte(ACCEL_ZOUT_H) << 8) | read_byte(ACCEL_ZOUT_L));
    acc->z = (float)(acc_raw) / ACCEL_FACTOR - az_offset;
}

void ReadGyro(AxesRaw *gyro)
{
    int16_t gyro_raw;
    // H:8bit shift, Link h and l
    gyro_raw = (int16_t)((int16_t)(read_byte(GYRO_XOUT_H) << 8) | read_byte(GYRO_XOUT_L));
    gyro->x = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f; // dps to rad/sec
    gyro_raw = (int16_t)((int16_t)(read_byte(GYRO_YOUT_H) << 8) | read_byte(GYRO_YOUT_L));
    gyro->y = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f; // dps to rad/sec
    gyro_raw = (int16_t)((int16_t)(read_byte(GYRO_ZOUT_H) << 8) | read_byte(GYRO_ZOUT_L));
    gyro->z = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f; // dps to rad/sec
}

void ReadMag(AxesRaw *mag)
{
}

void ReadSensor(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag)
{
    ReadAcc(acc);
    ReadGyro(gyro);
    ReadMag(mag);
}