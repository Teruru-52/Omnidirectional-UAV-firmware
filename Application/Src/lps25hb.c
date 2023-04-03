/*
 *  lps25hb.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "lps25hb.h"

uint8_t read_byte_LPS25HB(uint8_t reg)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg | 0x80;
    tx_data[1] = 0x00; // dummy

    Write_GPIO(SPI3_CS, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI3_CS, GPIO_PIN_SET);

    return rx_data[1];
}

void write_byte_LPS25HB(uint8_t reg, uint8_t data)
{
    uint8_t rx_data[2];
    uint8_t tx_data[2];

    tx_data[0] = reg & 0x7F;
    //   tx_data[0] = reg | 0x00;
    tx_data[1] = data; // write data

    Write_GPIO(SPI3_CS, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, tx_data, rx_data, 2, 10);
    Write_GPIO(SPI3_CS, GPIO_PIN_SET);
}

void CheckWhoAmI_LPS25HB()
{
    uint8_t who_am_i;

    who_am_i = read_byte_LPS25HB(LPS25HB_WHO_AM_I);  // read who am i
    printf("lps25hb who_am_i = 0x%x\r\n", who_am_i); // check who am i value
    HAL_Delay(10);
    while (who_am_i != 0xBD)
    {
        who_am_i = read_byte_LPS25HB(LPS25HB_WHO_AM_I);
        printf("lps25hb who_am_i = 0x%x\r\n", who_am_i);
        HAL_Delay(20);
    }
}

void Initialize_LPS25HB()
{
    Write_GPIO(SPI3_CS, GPIO_PIN_SET);
    __HAL_SPI_ENABLE(&hspi3); // clockが動かないように、あらかじめEnableにしておく
    HAL_Delay(100);           // wait start up

    CheckWhoAmI_LPS25HB();

    write_byte_LPS25HB(CTRL_REG2, 0x04); // reset software
    while (read_byte_LPS25HB(CTRL_REG2))
        ;                                // check reset
    HAL_Delay(100);                      // long wait between LPS25HB software reset
    write_byte_LPS25HB(CTRL_REG2, 0x80); // reboot memory content
    while (read_byte_LPS25HB(CTRL_REG2))
        ;                                // check reboot
    HAL_Delay(100);                      // long wait between LPS25HB memory reboot
    write_byte_LPS25HB(CTRL_REG1, 0x92); // set active mode (1Hz)
    HAL_Delay(100);                      // long wait between LPS25HB mode changes

    printf("finished LPS25HB initialization\n");
}

void ReadPressure(float *pressure)
{
    int32_t press_raw;
    // H:8bit shift, Link h and l
    press_raw = (int32_t)(read_byte_LPS25HB(PRESS_OUT_H));
    press_raw = (press_raw << 8) | (int32_t)(read_byte_LPS25HB(PRESS_OUT_L));
    press_raw = (press_raw << 8) | (int32_t)(read_byte_LPS25HB(PRESS_OUT_XL));
    float press = (float)(press_raw) / 4096.0f;

    *pressure = press;
}