/*
 *  mpu9250.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "mpu9250.h"

float bx = 0;
float by = 0;
float bz = 0;

float ax_offset = -0.001176f, ay_offset = -0.001515f, az_offset = -0.045435f;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
// at my apartment
float mx_offset = -1274.5f, my_offset = 27.4787f, mz_offset = 600.4536f;

uint8_t buffer[21];
uint8_t asa[3];

__weak void MPU9250_OnActivate()
{
}

static inline void MPU9250_Activate()
{
    MPU9250_OnActivate();
    Write_GPIO(SPI1_CS, GPIO_PIN_RESET);
}

static inline void MPU9250_Deactivate()
{
    Write_GPIO(SPI1_CS, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
    uint8_t receivedbyte = 0;
    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&Byte, (uint8_t *)&receivedbyte, 1, 0x1000) != HAL_OK)
    {
        return -1;
    }
    else
    {
    }
    return receivedbyte;
}

void MPU_SPI_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
    MPU9250_Activate();
    SPIx_WriteRead(WriteAddr);
    while (NumByteToWrite >= 0x01)
    {
        SPIx_WriteRead(*pBuffer);
        NumByteToWrite--;
        pBuffer++;
    }
    MPU9250_Deactivate();
}

void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
    MPU9250_Activate();
    uint8_t data = ReadAddr | 0x80;
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    HAL_SPI_Receive(&hspi1, pBuffer, NumByteToRead, 10);
    MPU9250_Deactivate();
}

/* writes a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data)
{
    MPU_SPI_Write(&data, subAddress, 1);
    HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    MPU_SPI_Read(dest, subAddress, count);
}

/* writes a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR);   // set slave 0 to the AK8963 and set for write
    writeRegister(I2C_SLV0_REG, subAddress);         // set the register to the desired AK8963 sub address
    writeRegister(I2C_SLV0_DO, data);                // store the data for write
    writeRegister(I2C_SLV0_CTRL, 0x80 | (uint8_t)1); // enable I2C and send 1 byte
}

/* reads registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t *dest)
{
    writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80); // set slave 0 to the AK8963 and set for read
    writeRegister(I2C_SLV0_REG, subAddress);              // set the register to the desired AK8963 sub address
    writeRegister(I2C_SLV0_CTRL, 0x80 | count);           // enable I2C and request the bytes
    readRegisters(EXT_SENS_DATA_00, count, dest);         // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

void CheckWhoAmI_MPU9250()
{
    readRegisters(WHO_AM_I, 1, buffer);               // read who am i
    printf("mpu9250 who_am_i = 0x%x\r\n", buffer[0]); // check who am i value
    HAL_Delay(10);
    while (buffer[0] != 0x71)
    {
        readRegisters(WHO_AM_I, 1, buffer);
        printf("mpu9250 who_am_i = 0x%x\r\n", buffer[0]);
        HAL_Delay(20);
    }
}

void CheckWhoAmI_AK8963()
{
    readAK8963Registers(AK8963_WHO_AM_I, 1, buffer); // read who am i
    printf("ak8963 who_am_i = 0x%x\r\n", buffer[0]); // check who am i value
    HAL_Delay(10);
    while (buffer[0] != 0x48)
    {
        readAK8963Registers(AK8963_WHO_AM_I, 1, buffer);
        printf("ak8963 who_am_i = 0x%x\r\n", buffer[0]);
        HAL_Delay(20);
    }
}

void InitializeIMU()
{
    Write_GPIO(SPI1_CS, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(&hspi1); // clockが動かないように、あらかじめEnableにしておく
    HAL_Delay(100);           // wait start up

    HAL_Delay(10);
    writeRegister(PWR_MGMT_1, 0x00); // select clock source to gyro(20MHz)
    HAL_Delay(10);
    writeRegister(USER_CTRL, 0x20); // enable I2C master mode
    HAL_Delay(10);
    writeRegister(I2C_MST_CTRL, 0x0D); // set the I2C bus speed to 400 kHz

    HAL_Delay(10);
    writeAK8963Register(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
    HAL_Delay(10);
    writeRegister(PWR_MGMT_1, 0x80); // reset the MPU9250
    HAL_Delay(10);
    writeAK8963Register(AK8963_CNTL2, 0x00); // reset the AK8963
    HAL_Delay(10);
    writeRegister(PWR_MGMT_1, 0x00); // select clock source to gyro(20MHz)

    HAL_Delay(10);
    CheckWhoAmI_MPU9250();

    HAL_Delay(10);
    writeRegister(PWR_MGMT_2, 0x00); // enable accelerometer and gyro
    HAL_Delay(10);
    writeRegister(CONFIG, 0x00); // set gyro bandwidth (250Hz)
    HAL_Delay(10);
    // writeRegister(CONFIG, 0x01); // set gyro bandwidth (184Hz)
    // HAL_Delay(10);
    writeRegister(GYRO_CONFIG, 0x18); // set gyro config (2000dps)
    HAL_Delay(10);
    writeRegister(ACCEL_CONFIG, 0x08); // set acc config (4g)
    HAL_Delay(10);
    // writeRegister(ACCEL_CONFIG2, 0x01); // set bandwidth (218.1Hz)
    // HAL_Delay(10);
    writeRegister(SMPLRT_DIV, 0x00); // setting the sample rate divider to 0
    HAL_Delay(10);
    writeRegister(USER_CTRL, 0x20); // enable I2C master mode
    HAL_Delay(10);
    writeRegister(I2C_MST_CTRL, 0x0D); // set the I2C bus speed to 400 kHz

    CheckWhoAmI_AK8963();

    /* get the magnetometer calibration */
    writeAK8963Register(AK8963_CNTL1, 0x00);  // set AK8963 to Power Down
    HAL_Delay(100);                           // long wait between AK8963 mode changes
    writeRegister(AK8963_CNTL1, 0x0F);        // set AK8963 to FUSE ROM access
    HAL_Delay(100);                           // long wait between AK8963 mode changes
    readAK8963Registers(AK8963_ASAX, 3, asa); // read the AK8963 ASA registers and compute magnetometer scale factors
    printf("asa_x = %d, asa_y = %d, asa_z = %d\n", asa[0], asa[1], asa[2]);
    HAL_Delay(10);
    writeAK8963Register(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
    HAL_Delay(100);                          // long wait between AK8963 mode changes
    // writeAK8963Register(AK8963_CNTL1, 0x11); // set AK8963 to 16 bit resolution, single measurement mode
    writeAK8963Register(AK8963_CNTL1, 0x16); // set AK8963 to 16 bit resolution, 100 Hz update rate
    HAL_Delay(100);
    writeRegister(PWR_MGMT_1, 0x00); // select clock source to gyro(20MHz)
    HAL_Delay(10);
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL, 7, buffer);

    printf("finished MPU9250 initialization\n");
}

void CalcAccOffset(AxesRaw *acc)
{
    AxesRaw acc_offset_sum = {0, 0, 0};
    for (int i = 0; i < max_cali_count; i++)
    {
        ReadRawAcc(acc);
        acc_offset_sum.x += acc->x;
        acc_offset_sum.y += acc->y;
        acc_offset_sum.z += acc->z;
    }
    ax_offset = acc_offset_sum.x / (float)(max_cali_count);
    ay_offset = acc_offset_sum.y / (float)(max_cali_count);
    // az_offset = acc_offset_sum.z / (float)(max_cali_count);
    az_offset = acc_offset_sum.z / (float)(max_cali_count) + 1.0f;
    printf("acc offset = %f,\t%f,\t%f\t\r\n", ax_offset, ay_offset, az_offset);
}

void CalcGyroOffset(AxesRaw *gyro)
{
    AxesRaw gyro_offset_sum = {0, 0, 0};
    for (int i = 0; i < max_cali_count; i++)
    {
        ReadRawGyro(gyro);
        gyro_offset_sum.x += gyro->x;
        gyro_offset_sum.y += gyro->y;
        gyro_offset_sum.z += gyro->z;
    }
    gx_offset = gyro_offset_sum.x / (float)(max_cali_count);
    gy_offset = gyro_offset_sum.y / (float)(max_cali_count);
    gz_offset = gyro_offset_sum.z / (float)(max_cali_count);
}

void SetInitialMag(AxesRaw *mag)
{
    // AxesRaw mag_sum = {0, 0, 0};
    // for (int i = 0; i < max_cali_count; i++)
    // {
    //     ReadRawMag(mag);
    //     mag_sum.x += (mag->x - mx_offset);
    //     mag_sum.y += (mag->y - my_offset);
    //     mag_sum.z += (mag->z - mz_offset);
    // }
    // bx = mag_sum.x / (float)(max_cali_count);
    // by = mag_sum.y / (float)(max_cali_count);
    // bz = mag_sum.z / (float)(max_cali_count);

    ReadRawMag(mag);
    bx = mag->x - mx_offset;
    by = mag->y - my_offset;
    bz = mag->z - mz_offset;

    float recipNorm = invSqrt(bx * bx + by * by + bz * bz);
    bx *= recipNorm;
    by *= recipNorm;
    bz *= recipNorm;
}

void ReadRawAcc(AxesRaw *acc)
{
    int16_t acc_raw;

    // grab the data from the MPU9250
    readRegisters(ACCEL_XOUT_H, 21, buffer);

    // combine into 16 bit values
    acc_raw = (((int16_t)buffer[0]) << 8) | buffer[1];
    acc->x = (float)(acc_raw) / ACCEL_FACTOR;
    acc_raw = (((int16_t)buffer[2]) << 8) | buffer[3];
    acc->y = (float)(acc_raw) / ACCEL_FACTOR;
    acc_raw = (((int16_t)buffer[4]) << 8) | buffer[5];
    acc->z = (float)(acc_raw) / ACCEL_FACTOR;
}

void ReadRawGyro(AxesRaw *gyro)
{
    int16_t gyro_raw;

    // grab the data from the MPU9250
    readRegisters(ACCEL_XOUT_H, 21, buffer);

    gyro_raw = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyro->x = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f; // dps to rad/sec
    gyro_raw = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyro->y = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f; // dps to rad/sec
    gyro_raw = (((int16_t)buffer[12]) << 8) | buffer[13];
    gyro->z = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f; // dps to rad/sec
}

void ReadRawMag(AxesRaw *mag)
{
    int16_t mag_raw;

    // grab the data from the MPU9250
    readRegisters(ACCEL_XOUT_H, 21, buffer);

    mag_raw = (((int16_t)buffer[17]) << 8) | buffer[16];
    mag->x = (float)(mag_raw) * ((float)(asa[1] - 128) / 256.0f + 1.0f);
    mag_raw = (((int16_t)buffer[15]) << 8) | buffer[14];
    mag->y = (float)(mag_raw) * ((float)(asa[0] - 128) / 256.0f + 1.0f);
    mag_raw = (((int16_t)buffer[19]) << 8) | buffer[18];
    mag->z = -(float)(mag_raw) * ((float)(asa[2] - 128) / 256.0f + 1.0f);
}

void ReadSensor(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag)
{
    int16_t acc_raw;
    int16_t gyro_raw;
    int16_t mag_raw;

    // grab the data from the MPU9250
    readRegisters(ACCEL_XOUT_H, 21, buffer);

    // combine into 16 bit values
    acc_raw = (((int16_t)buffer[0]) << 8) | buffer[1];
    acc->x = (float)(acc_raw) / ACCEL_FACTOR - ax_offset;
    acc_raw = (((int16_t)buffer[2]) << 8) | buffer[3];
    acc->y = (float)(acc_raw) / ACCEL_FACTOR - ay_offset;
    acc_raw = (((int16_t)buffer[4]) << 8) | buffer[5];
    acc->z = (float)(acc_raw) / ACCEL_FACTOR - az_offset;

    gyro_raw = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyro->x = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f - gx_offset; // dps to rad/sec
    gyro_raw = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyro->y = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f - gy_offset; // dps to rad/sec
    gyro_raw = (((int16_t)buffer[12]) << 8) | buffer[13];
    gyro->z = (float)(gyro_raw) / GYRO_FACTOR * M_PI / 180.0f - gz_offset; // dps to rad/sec

    mag_raw = (((int16_t)buffer[17]) << 8) | buffer[16];
    mag->x = (float)(mag_raw) * ((float)(asa[1] - 128) / 256.0f + 1.0f) - mx_offset;
    mag_raw = (((int16_t)buffer[15]) << 8) | buffer[14];
    mag->y = (float)(mag_raw) * ((float)(asa[0] - 128) / 256.0f + 1.0f) - my_offset;
    mag_raw = (((int16_t)buffer[19]) << 8) | buffer[18];
    mag->z = -(float)(mag_raw) * ((float)(asa[2] - 128) / 256.0f + 1.0f) - mz_offset;
}