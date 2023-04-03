/*
 *  speaker.c
 *
 *  Created on: April 1st, 2023
 *      Author: Reiji Terunuma
 */

#include "speaker.h"

void Beep()
{
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 20);
    HAL_Delay(50);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
}