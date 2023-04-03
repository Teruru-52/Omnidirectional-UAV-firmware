/*
 *  ahrs.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "ahrs.h"

float dt;
float half_dt;
float beta;
float zeta;
float _2g;
float _2mn;
float _2me;
float _2md;

void InitializeAHRS(AHRS_State *ahrs)
{
    ahrs->q.q0 = 1.0f;
    ahrs->q.q1 = 0.0f;
    ahrs->q.q2 = 0.0f;
    ahrs->q.q3 = 0.0f;
    ahrs->gx_offset = 0.0f;
    ahrs->gy_offset = 0.0f;
    ahrs->gz_offset = 0.0f;

    dt = 0.01f;
    half_dt = dt * 0.5f;
    beta = 0.1f;
    zeta = 0.004f;
    _2g = 2.0f * g;
    _2mn = 2.0f * mn;
    _2me = 2.0f * me;
    _2md = 2.0f * md;

    float coeff_transA[49];
    float coeff_C[42];
    coeff_estX[0] = 1.0f;
    coeff_estX[1] = 0.0f;
    coeff_estX[2] = 0.0f;
    coeff_estX[3] = 0.0f;
    coeff_estX[4] = 0.0f;
    coeff_estX[5] = 0.0f;
    coeff_estX[6] = 0.0f;
    float coeff_barX[7];
    float coeff_gm[6];
    coeff_gm[0] = 0.0f;
    coeff_gm[1] = 0.0f;
    coeff_gm[2] = -g;
    coeff_gm[3] = mn;
    coeff_gm[4] = me;
    coeff_gm[5] = md;

    float coeff_V[49] = {
        1e-7f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1e-7f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1e-7f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1e-7f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1e-7f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-7f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-7f};

    float coeff_W[36] = {
        1e+1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1e+1f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1e+1f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1e-2f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1e-2f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-2f};

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

    float coeff_P[49] = {
        1e-5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1e-5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1e-5f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1e-5f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1e-3f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-3f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-3f};

    float coeff_GtransC[49];
    float coeff_GtransCbarP[49];

    arm_mat_init_f32(&mat_transA, 7, 7, coeff_transA);
    arm_mat_init_f32(&mat_C, 6, 7, coeff_C);
    arm_mat_init_f32(&mat_estX, 7, 1, coeff_estX);
    arm_mat_init_f32(&mat_barX, 7, 1, coeff_barX);
    arm_mat_init_f32(&mat_gm, 6, 1, coeff_gm);
    arm_mat_init_f32(&mat_V, 7, 7, coeff_V);
    arm_mat_init_f32(&mat_W, 6, 6, coeff_W);
    arm_mat_init_f32(&mat_barP, 7, 7, coeff_barP);
    arm_mat_init_f32(&mat_AP, 7, 7, coeff_AP);
    arm_mat_init_f32(&mat_APtransA, 7, 7, coeff_APtransA);
    arm_mat_init_f32(&mat_G, 7, 6, coeff_G);
    arm_mat_init_f32(&mat_Gnum, 7, 6, coeff_Gnum);
    arm_mat_init_f32(&mat_Gden, 6, 6, coeff_Gden);
    arm_mat_init_f32(&mat_transCbarP, 6, 7, coeff_transCbarP);
    arm_mat_init_f32(&mat_transCbarPC, 6, 6, coeff_transCbarPC);
    arm_mat_init_f32(&mat_invGden, 6, 6, coeff_invGden);
    arm_mat_init_f32(&mat_H, 6, 1, coeff_H);
    arm_mat_init_f32(&mat_E, 6, 1, coeff_E);
    arm_mat_init_f32(&mat_P, 7, 7, coeff_P);
    arm_mat_init_f32(&mat_GtransC, 7, 7, coeff_GtransC);
    arm_mat_init_f32(&mat_GtransCbarP, 7, 7, coeff_GtransCbarP);
}

void UpdateMadgwickFilter(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs)
{
    float recipNorm;
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    Quaternion dq;
    float hx, hy;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mag->x == 0.0f) && (mag->y == 0.0f) && (mag->z == 0.0f))
    {
        UpdateMadgwickFilterIMU(acc, gyro, ahrs);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((acc->x == 0.0f) && (acc->y == 0.0f) && (acc->z == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
        acc->x *= recipNorm;
        acc->y *= recipNorm;
        acc->z *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mag->x * mag->x + mag->y * mag->y + mag->z * mag->z);
        mag->x *= recipNorm;
        mag->y *= recipNorm;
        mag->z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0mx = 2.0f * ahrs->q.q0 * mag->x;
        float _2q0my = 2.0f * ahrs->q.q0 * mag->y;
        float _2q0mz = 2.0f * ahrs->q.q0 * mag->z;
        float _2q1mx = 2.0f * ahrs->q.q1 * mag->x;
        float _2q0 = 2.0f * ahrs->q.q0;
        float _2q1 = 2.0f * ahrs->q.q1;
        float _2q2 = 2.0f * ahrs->q.q2;
        float _2q3 = 2.0f * ahrs->q.q3;
        float _2q0q2 = 2.0f * ahrs->q.q0 * ahrs->q.q2;
        float _2q2q3 = 2.0f * ahrs->q.q2 * ahrs->q.q3;
        float q0q0 = ahrs->q.q0 * ahrs->q.q0;
        float q0q1 = ahrs->q.q0 * ahrs->q.q1;
        float q0q2 = ahrs->q.q0 * ahrs->q.q2;
        float q0q3 = ahrs->q.q0 * ahrs->q.q3;
        float q1q1 = ahrs->q.q1 * ahrs->q.q1;
        float q1q2 = ahrs->q.q1 * ahrs->q.q2;
        float q1q3 = ahrs->q.q1 * ahrs->q.q3;
        float q2q2 = ahrs->q.q2 * ahrs->q.q2;
        float q2q3 = ahrs->q.q2 * ahrs->q.q3;
        float q3q3 = ahrs->q.q3 * ahrs->q.q3;

        // Reference direction of Earth's magnetic field
        hx = mag->x * q0q0 - _2q0my * ahrs->q.q3 + _2q0mz * ahrs->q.q2 + mag->x * q1q1 + _2q1 * mag->y * ahrs->q.q2 + _2q1 * mag->z * ahrs->q.q3 - mag->x * q2q2 - mag->x * q3q3;
        hy = _2q0mx * ahrs->q.q3 + mag->y * q0q0 - _2q0mz * ahrs->q.q1 + _2q1mx * ahrs->q.q2 - mag->y * q1q1 + mag->y * q2q2 + _2q2 * mag->z * ahrs->q.q3 - mag->y * q3q3;
        float _2bx = Sqrt(hx * hx + hy * hy);
        float _2bz = -_2q0mx * ahrs->q.q2 + _2q0my * ahrs->q.q1 + mag->z * q0q0 + _2q1mx * ahrs->q.q3 - mag->z * q1q1 + _2q2 * mag->y * ahrs->q.q3 - mag->z * q2q2 + mag->z * q3q3;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q1 * (2.0f * q0q1 + _2q2q3 - acc->y) - _2bz * ahrs->q.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (-_2bx * ahrs->q.q3 + _2bz * ahrs->q.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + _2bx * ahrs->q.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q0 * (2.0f * q0q1 + _2q2q3 - acc->y) - 4.0f * ahrs->q.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - acc->z) + _2bz * ahrs->q.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (_2bx * ahrs->q.q2 + _2bz * ahrs->q.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + (_2bx * ahrs->q.q3 - _4bz * ahrs->q.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q3 * (2.0f * q0q1 + _2q2q3 - acc->y) - 4.0f * ahrs->q.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - acc->z) + (-_4bx * ahrs->q.q2 - _2bz * ahrs->q.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (_2bx * ahrs->q.q1 + _2bz * ahrs->q.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + (_2bx * ahrs->q.q0 - _4bz * ahrs->q.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q2 * (2.0f * q0q1 + _2q2q3 - acc->y) + (-_4bx * ahrs->q.q3 + _2bz * ahrs->q.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (-_2bx * ahrs->q.q0 + _2bz * ahrs->q.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + _2bx * ahrs->q.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
    }

    ahrs->gx = gyro->x;
    ahrs->gy = gyro->y;
    ahrs->gz = gyro->z;

    // compute and remove the gyroscope baises
    ahrs->gx_offset += 2.0f * (ahrs->q.q0 * s1 - ahrs->q.q1 * s0 - ahrs->q.q2 * s3 + ahrs->q.q3 * s2) * dt;
    ahrs->gy_offset += 2.0f * (ahrs->q.q0 * s2 + ahrs->q.q1 * s3 - ahrs->q.q2 * s0 - ahrs->q.q3 * s1) * dt;
    ahrs->gz_offset += 2.0f * (ahrs->q.q0 * s3 - ahrs->q.q1 * s2 + ahrs->q.q2 * s1 - ahrs->q.q3 * s0) * dt;

    ahrs->gx -= zeta * ahrs->gx_offset;
    ahrs->gy -= zeta * ahrs->gy_offset;
    ahrs->gz -= zeta * ahrs->gz_offset;

    // Rate of change of quaternion from gyroscope
    dq.q0 = 0.5f * (-ahrs->q.q1 * ahrs->gx - ahrs->q.q2 * ahrs->gy - ahrs->q.q3 * ahrs->gz);
    dq.q1 = 0.5f * (ahrs->q.q0 * ahrs->gx + ahrs->q.q2 * ahrs->gz - ahrs->q.q3 * ahrs->gy);
    dq.q2 = 0.5f * (ahrs->q.q0 * ahrs->gy - ahrs->q.q1 * ahrs->gz + ahrs->q.q3 * ahrs->gx);
    dq.q3 = 0.5f * (ahrs->q.q0 * ahrs->gz + ahrs->q.q1 * ahrs->gy - ahrs->q.q2 * ahrs->gx);

    // Apply feedback step
    dq.q0 -= beta * s0;
    dq.q1 -= beta * s1;
    dq.q2 -= beta * s2;
    dq.q3 -= beta * s3;

    // Integrate rate of change of quaternion to yield quaternion
    ahrs->q.q0 += dq.q0 * dt;
    ahrs->q.q1 += dq.q1 * dt;
    ahrs->q.q2 += dq.q2 * dt;
    ahrs->q.q3 += dq.q3 * dt;
    QuaternionNorm(&(ahrs->q));
}

void UpdateMadgwickFilterIMU(AxesRaw *acc, AxesRaw *gyro, AHRS_State *ahrs)
{
    float recipNorm;
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    Quaternion dq;

    // Rate of change of quaternion from gyroscope
    dq.q0 = 0.5f * (-ahrs->q.q1 * gyro->x - ahrs->q.q2 * gyro->y - ahrs->q.q3 * gyro->z);
    dq.q1 = 0.5f * (ahrs->q.q0 * gyro->x + ahrs->q.q2 * gyro->z - ahrs->q.q3 * gyro->y);
    dq.q2 = 0.5f * (ahrs->q.q0 * gyro->y - ahrs->q.q1 * gyro->z + ahrs->q.q3 * gyro->x);
    dq.q3 = 0.5f * (ahrs->q.q0 * gyro->z + ahrs->q.q1 * gyro->y - ahrs->q.q2 * gyro->x);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((acc->x == 0.0f) && (acc->y == 0.0f) && (acc->z == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
        acc->x *= recipNorm;
        acc->y *= recipNorm;
        acc->z *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * ahrs->q.q0;
        float _2q1 = 2.0f * ahrs->q.q1;
        float _2q2 = 2.0f * ahrs->q.q2;
        float _2q3 = 2.0f * ahrs->q.q3;
        float _4q0 = 4.0f * ahrs->q.q0;
        float _4q1 = 4.0f * ahrs->q.q1;
        float _4q2 = 4.0f * ahrs->q.q2;
        float _8q1 = 8.0f * ahrs->q.q1;
        float _8q2 = 8.0f * ahrs->q.q2;
        float q0q0 = ahrs->q.q0 * ahrs->q.q0;
        float q1q1 = ahrs->q.q1 * ahrs->q.q1;
        float q2q2 = ahrs->q.q2 * ahrs->q.q2;
        float q3q3 = ahrs->q.q3 * ahrs->q.q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * acc->x + _4q0 * q1q1 - _2q1 * acc->y;
        s1 = _4q1 * q3q3 - _2q3 * acc->x + 4.0f * q0q0 * ahrs->q.q1 - _2q0 * acc->y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * acc->z;
        s2 = 4.0f * q0q0 * ahrs->q.q2 + _2q0 * acc->x + _4q2 * q3q3 - _2q3 * acc->y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * acc->z;
        s3 = 4.0f * q1q1 * ahrs->q.q3 - _2q1 * acc->x + 4.0f * q2q2 * ahrs->q.q3 - _2q2 * acc->y;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        dq.q0 -= beta * s0;
        dq.q1 -= beta * s1;
        dq.q2 -= beta * s2;
        dq.q3 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    ahrs->q.q0 += dq.q0 * dt;
    ahrs->q.q1 += dq.q1 * dt;
    ahrs->q.q2 += dq.q2 * dt;
    ahrs->q.q3 += dq.q3 * dt;
    QuaternionNorm(&(ahrs->q));
}

void UpdateEKF(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs)
{
    float coeff_q0 = ahrs->q.q0 * half_dt;
    float coeff_q1 = ahrs->q.q1 * half_dt;
    float coeff_q2 = ahrs->q.q2 * half_dt;
    float coeff_q3 = ahrs->q.q3 * half_dt;
    float coeff_gx = (gyro->x - ahrs->gx_offset) * half_dt;
    float coeff_gy = (gyro->y - ahrs->gy_offset) * half_dt;
    float coeff_gz = (gyro->z - ahrs->gz_offset) * half_dt;

    float coeff_F[49] = {
        1.0f, -coeff_gx, -coeff_gy, -coeff_gz, 0.0f, 0.0f, 0.0f,
        coeff_gx, 1.0f, coeff_gz, -coeff_gy, 0.0f, 0.0f, 0.0f,
        coeff_gy, -coeff_gz, 1.0f, coeff_gx, 0.0f, 0.0f, 0.0f,
        coeff_gz, coeff_gy, -coeff_gx, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    arm_mat_init_f32(&mat_F, 7, 7, coeff_F);

    arm_mat_mult_f32(&mat_F, &mat_estX, &mat_barX);

    ahrs->q.q0 = coeff_barX[0];
    ahrs->q.q1 = coeff_barX[1];
    ahrs->q.q2 = coeff_barX[2];
    ahrs->q.q3 = coeff_barX[3];
    QuaternionNorm(&(ahrs->q));
    coeff_barX[0] = ahrs->q.q0;
    coeff_barX[1] = ahrs->q.q1;
    coeff_barX[2] = ahrs->q.q2;
    coeff_barX[3] = ahrs->q.q3;

    float coeff_A[49] = {
        1.0f, -coeff_gx, -coeff_gy, -coeff_gz, coeff_q1, coeff_q2, coeff_q3,
        coeff_gx, 1.0f, coeff_gz, -coeff_gy, -coeff_q0, coeff_q3, -coeff_q2,
        coeff_gy, -coeff_gz, 1.0f, coeff_gx, -coeff_q3, -coeff_q0, coeff_q1,
        coeff_gz, coeff_gy, -coeff_gx, 1.0f, coeff_q2, -coeff_q1, -coeff_q0,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    arm_mat_init_f32(&mat_A, 7, 7, coeff_A);
    arm_mat_trans_f32(&mat_A, &mat_transA);

    // auxiliary variables to reduce number of repeated operations
    float _2gq0 = _2g * ahrs->q.q0;
    float _2gq1 = _2g * ahrs->q.q1;
    float _2gq2 = _2g * ahrs->q.q2;
    float _2gq3 = _2g * ahrs->q.q3;
    float _2mnq0 = _2mn * ahrs->q.q0;
    float _2mnq1 = _2mn * ahrs->q.q1;
    float _2mnq2 = _2mn * ahrs->q.q2;
    float _2mnq3 = _2mn * ahrs->q.q3;
    float _2meq0 = _2me * ahrs->q.q0;
    float _2meq1 = _2me * ahrs->q.q1;
    float _2meq2 = _2me * ahrs->q.q2;
    float _2meq3 = _2me * ahrs->q.q3;
    float _2mdq0 = _2md * ahrs->q.q0;
    float _2mdq1 = _2md * ahrs->q.q1;
    float _2mdq2 = _2md * ahrs->q.q2;
    float _2mdq3 = _2md * ahrs->q.q3;

    float coeff_transC[42] = {
        _2gq2, -_2gq3, _2gq0, -_2gq1, 0.0f, 0.0f, 0.0f,
        -_2gq1, -_2gq0, -_2gq3, -_2gq2, 0.0f, 0.0f, 0.0f,
        -_2gq0, _2gq1, _2gq2, -_2gq3, 0.0f, 0.0f, 0.0f,
        (_2mnq0 + _2meq3 - _2mdq2), (_2mnq1 + _2meq2 + _2mdq3), (-_2mnq2 + _2meq1 - _2mdq0), (-_2mnq3 + _2meq0 + _2mdq1), 0.0f, 0.0f, 0.0f,
        (-_2mnq3 + _2meq0 + _2mdq1), (_2mnq2 - _2meq1 + _2mdq0), (_2mnq1 + _2meq2 + _2mdq3), (-_2mnq0 - _2meq3 + _2mdq2), 0.0f, 0.0f, 0.0f,
        (_2mnq2 - _2meq1 + _2mdq0), (_2mnq3 - _2meq0 - _2mdq1), (_2mnq0 + _2meq3 - _2mdq2), (_2mnq1 + _2meq2 + _2mdq3), 0.0f, 0.0f, 0.0f};
    arm_mat_init_f32(&mat_transC, 6, 7, coeff_transC);
    arm_mat_trans_f32(&mat_transC, &mat_C);

    // calculate covariance matrix
    arm_mat_mult_f32(&mat_A, &mat_P, &mat_AP);
    arm_mat_mult_f32(&mat_AP, &mat_transA, &mat_APtransA);
    arm_mat_add_f32(&mat_APtransA, &mat_V, &mat_barP);

    // calculate Kalman gain
    arm_mat_mult_f32(&mat_barP, &mat_C, &mat_Gnum);
    arm_mat_mult_f32(&mat_transC, &mat_barP, &mat_transCbarP);
    arm_mat_mult_f32(&mat_transCbarP, &mat_C, &mat_transCbarPC);
    arm_mat_add_f32(&mat_transCbarPC, &mat_W, &mat_Gden);
    arm_mat_inverse_f32(&mat_Gden, &mat_invGden);
    arm_mat_mult_f32(&mat_Gnum, &mat_invGden, &mat_G);

    // auxiliary variables to reduce number of repeated operations
    float q0q0 = ahrs->q.q0 * ahrs->q.q0;
    float q0q1 = ahrs->q.q0 * ahrs->q.q1;
    float q0q2 = ahrs->q.q0 * ahrs->q.q2;
    float q0q3 = ahrs->q.q0 * ahrs->q.q3;
    float q1q1 = ahrs->q.q1 * ahrs->q.q1;
    float q1q2 = ahrs->q.q1 * ahrs->q.q2;
    float q1q3 = ahrs->q.q1 * ahrs->q.q3;
    float q2q2 = ahrs->q.q2 * ahrs->q.q2;
    float q2q3 = ahrs->q.q2 * ahrs->q.q3;
    float q3q3 = ahrs->q.q3 * ahrs->q.q3;

    float coeff_Y[6] = {acc->x, acc->y, acc->z, mag->x, mag->y, mag->z};
    arm_mat_init_f32(&mat_Y, 6, 1, coeff_Y);

    float coeff_invR[9] = {
        2.0f * (q0q0 + q1q1) - 1.0f, 2.0f * (q1q2 + q0q3), 2.0f * (q1q3 - q0q2),
        2.0f * (q1q2 - q0q3), 2.0f * (q0q0 + q2q2) - 1.0f, 2.0f * (q2q3 + q0q1),
        2.0f * (q1q3 + q0q2), 2.0f * (q2q3 - q0q1), 2.0f * (q0q0 + q3q3) - 1.0f};
    arm_mat_init_f32(&mat_invR, 3, 3, coeff_invR);

    arm_mat_mult_f32(&mat_invR, &mat_gm, &mat_H);
    arm_mat_sub_f32(&mat_Y, &mat_H, &mat_E);
    arm_mat_mult_f32(&mat_G, &mat_E, &mat_GE);
    arm_mat_add_f32(&mat_barX, &mat_GE, &mat_estX);

    ahrs->q.q0 = coeff_estX[0];
    ahrs->q.q1 = coeff_estX[1];
    ahrs->q.q2 = coeff_estX[2];
    ahrs->q.q3 = coeff_estX[3];
    QuaternionNorm(&(ahrs->q));
    coeff_estX[0] = ahrs->q.q0;
    coeff_estX[1] = ahrs->q.q1;
    coeff_estX[2] = ahrs->q.q2;
    coeff_estX[3] = ahrs->q.q3;

    arm_mat_mult_f32(&mat_G, &mat_transC, &mat_GtransC);
    arm_mat_mult_f32(&mat_GtransC, &mat_barP, &mat_GtransCbarP);
    arm_mat_sub_f32(&mat_barP, &mat_GtransCbarP, &mat_P);

    ahrs->gx_offset = coeff_estX[4];
    ahrs->gy_offset = coeff_estX[5];
    ahrs->gz_offset = coeff_estX[6];
    ahrs->gx = gyro->x - ahrs->gx_offset;
    ahrs->gy = gyro->y - ahrs->gy_offset;
    ahrs->gz = gyro->z - ahrs->gz_offset;
}