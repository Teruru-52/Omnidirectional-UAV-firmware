/*
 *  lps25hb.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "lps25hb.h"

// uint8_t read_byte(uint8_t reg)
// {
//     uint8_t rx_data[2];
//     uint8_t tx_data[2];

//     tx_data[0] = reg | 0x80;
//     tx_data[1] = 0x00; // dummy

//     Write_GPIO(SPI3_CS, GPIO_PIN_RESET);
//     HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
//     Write_GPIO(SPI3_CS, GPIO_PIN_SET);

//     return rx_data[1];
// }

// void write_byte(uint8_t reg, uint8_t data)
// {
//     uint8_t rx_data[2];
//     uint8_t tx_data[2];

//     tx_data[0] = reg & 0x7F;
//     //   tx_data[0] = reg | 0x00;
//     tx_data[1] = data; // write data

//     Write_GPIO(SPI3_CS, GPIO_PIN_RESET);
//     HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 10);
//     Write_GPIO(SPI3_CS, GPIO_PIN_SET);
// }