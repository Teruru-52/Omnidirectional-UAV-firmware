/*
 *  lps25hb.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __LPS25HB_H_
#define __LPS25HB_H_

#include "main.h"

#define LPS25HB_WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define PRESS_OUT_XL 0x28
#define PRESS_OUT_L 0x29
#define PRESS_OUT_H 0x2A
#define RPDS_L 0x39
#define RPDS_H 0x3A

uint8_t read_byte_LPS25HB(uint8_t reg);
void write_byte_LPS25HB(uint8_t reg, uint8_t data);

void CheckWhoAmI_LPS25HB();
void Initialize_LPS25HB();
void ReadPressure(float *pressure);

#endif /* __LPS25HB_H_ */