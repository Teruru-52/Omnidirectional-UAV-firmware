/*
 *  ahrs.h
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#ifndef __AHRS_H_
#define __AHRS_H_

#include "main.h"
#include "mpu9250.h"
#include "quaternion.h"

typedef struct
{
    Quaternion q;     // Current altitude
    float gx, gy, gz; // Current angle rate @ body frame
    float gx_offset, gy_offset, gz_offset;
} AHRS_State;

/* matrixes in Externded Kalman Filter */
float coeff_estX[7];
float coeff_barX[7];

arm_matrix_instance_f32 mat_transA;
arm_matrix_instance_f32 mat_C;
arm_matrix_instance_f32 mat_estX;
arm_matrix_instance_f32 mat_barX;
arm_matrix_instance_f32 mat_gm;
arm_matrix_instance_f32 mat_V;
arm_matrix_instance_f32 mat_W;
arm_matrix_instance_f32 mat_barP;
arm_matrix_instance_f32 mat_AP;
arm_matrix_instance_f32 mat_APtransA;
arm_matrix_instance_f32 mat_G;
arm_matrix_instance_f32 mat_Gnum;
arm_matrix_instance_f32 mat_Gden;
arm_matrix_instance_f32 mat_invGden;
arm_matrix_instance_f32 mat_transCbarP;
arm_matrix_instance_f32 mat_transCbarPC;
arm_matrix_instance_f32 mat_H;
arm_matrix_instance_f32 mat_E;
arm_matrix_instance_f32 mat_GE;
arm_matrix_instance_f32 mat_P;
arm_matrix_instance_f32 mat_GtransC;
arm_matrix_instance_f32 mat_GtransCbarP;

arm_matrix_instance_f32 mat_A;
arm_matrix_instance_f32 mat_F;
arm_matrix_instance_f32 mat_transC;
arm_matrix_instance_f32 mat_Y;
arm_matrix_instance_f32 mat_invR; // invR = transR

void InitializeEKF(AHRS_State *ahrs);
void UpdateEKF(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs);

#endif /* __AHRS_H_ */