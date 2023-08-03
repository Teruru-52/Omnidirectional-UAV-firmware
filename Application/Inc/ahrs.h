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
#include "kalman.h"

typedef struct
{
    Quaternion q;     // Current altitude
    AxesRaw euler;    // Current altitude
    float gx, gy, gz; // Current angle rate @ body frame
    float gx_offset, gy_offset, gz_offset;
} AHRS_State;

/* matrixes in Madgwick Filter */
float coeff_nabla_fg[4];
float coeff_nabla_fb[4];

arm_matrix_instance_f32 mat_nabla_fg;
arm_matrix_instance_f32 mat_fg;
arm_matrix_instance_f32 mat_transJg;

arm_matrix_instance_f32 mat_nabla_fb;
arm_matrix_instance_f32 mat_fb;
arm_matrix_instance_f32 mat_transJb;

/* matrixes in Externded Kalman Filter */
float coeff_estX[7];
float coeff_barX[7];
float coeff_gm[6];

float coeff_transA[49];
float coeff_C[42];
float coeff_barP[49];
float coeff_AP[49];
float coeff_APtransA[49];
float coeff_G[42];
float coeff_Gnum[42];
float coeff_Gden[36];
float coeff_transCbarP[42];
float coeff_transCbarPC[36];
float coeff_invGden[36];
float coeff_H[6];
float coeff_E[6];
float coeff_GE[7];

float coeff_GtransC[49];
float coeff_GtransCbarP[49];

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

/* variables in Complimantary Filter */
float roll_filt;
float pitch_filt;
float yaw_filt;

void InitializeAHRS(AHRS_State *ahrs);
// void TestMatrix();
void UpdateAHRS(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs);
void UpdateMadgwickFilter(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs);
void UpdateMadgwickFilterIMU(AxesRaw *acc, AxesRaw *gyro, AHRS_State *ahrs);
void CalcNabla_fg(AxesRaw *acc, AHRS_State *ahrs);
void CalcNabla_fb(AxesRaw *mag, AHRS_State *ahrs);
void UpdateEKF(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs);
void UpdateKF(AxesRaw *acc, AxesRaw *gyro, AHRS_State *ahrs); // only pitch angle
void UpdateComplimentaryFilter(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs);

void Convertq2Euler(AHRS_State *ahrs);
void ConvertEuler2q(AHRS_State *ahrs);
#endif /* __AHRS_H_ */