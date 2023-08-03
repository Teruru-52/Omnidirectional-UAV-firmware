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
float _2bx;
float _2by;
float _2bz;
float kappa;

const float32_t coeff_V[49] = {
    1e-3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1e-3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1e-3f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1e-3f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1e-3f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-3f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-3f};

const float32_t coeff_W[36] = {
    1e-1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1e-1f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1e-1f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1e-2f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1e-2f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-2f};

const float32_t coeff_P[49] = {
    1e-3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1e-3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1e-3f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1e-3f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1e-3f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-3f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1e-3f};

void InitializeAHRS(AHRS_State *ahrs)
{
    ahrs->q.q0 = 1.0f;
    ahrs->q.q1 = 0.0f;
    ahrs->q.q2 = 0.0f;
    ahrs->q.q3 = 0.0f;
    ahrs->euler.x = 0.0f;
    ahrs->euler.y = 0.0f;
    ahrs->euler.z = 0.0f;
    ahrs->gx_offset = 0.0f;
    ahrs->gy_offset = 0.0f;
    ahrs->gz_offset = 0.0f;

    dt = 0.01f;
    half_dt = dt * 0.5f;
    beta = 0.05f;
    zeta = 0.004f;
    _2bx = 2.0f * bx;
    _2by = 2.0f * by;
    _2bz = 2.0f * bz;

    arm_mat_init_f32(&mat_nabla_fg, 4, 1, coeff_nabla_fg);
    arm_mat_init_f32(&mat_nabla_fb, 4, 1, coeff_nabla_fb);

    coeff_estX[0] = 1.0f;
    coeff_estX[1] = 0.0f;
    coeff_estX[2] = 0.0f;
    coeff_estX[3] = 0.0f;
    coeff_estX[4] = 0.0f;
    coeff_estX[5] = 0.0f;
    coeff_estX[6] = 0.0f;

    // float coeff_gm[6] = {0.0f, 0.0f, -g, bx, by, bz};
    coeff_gm[0] = 0.0f;
    coeff_gm[1] = 0.0f;
    coeff_gm[2] = -1.0f;
    coeff_gm[3] = bx;
    coeff_gm[4] = by;
    coeff_gm[5] = bz;

    // arm_mat_init_f32(&mat_transA, 7, 7, coeff_transA);
    // arm_mat_init_f32(&mat_C, 7, 6, coeff_C);
    // arm_mat_init_f32(&mat_estX, 7, 1, coeff_estX);
    // arm_mat_init_f32(&mat_barX, 7, 1, coeff_barX);
    // arm_mat_init_f32(&mat_gm, 6, 1, coeff_gm);
    // arm_mat_init_f32(&mat_V, 7, 7, (float32_t *)coeff_V);
    // arm_mat_init_f32(&mat_W, 6, 6, (float32_t *)coeff_W);
    // arm_mat_init_f32(&mat_barP, 7, 7, coeff_barP);
    // arm_mat_init_f32(&mat_AP, 7, 7, coeff_AP);
    // arm_mat_init_f32(&mat_APtransA, 7, 7, coeff_APtransA);
    // arm_mat_init_f32(&mat_G, 7, 6, coeff_G);
    // arm_mat_init_f32(&mat_Gnum, 7, 6, coeff_Gnum);
    // arm_mat_init_f32(&mat_Gden, 6, 6, coeff_Gden);
    // arm_mat_init_f32(&mat_transCbarP, 6, 7, coeff_transCbarP);
    // arm_mat_init_f32(&mat_transCbarPC, 6, 6, coeff_transCbarPC);
    // arm_mat_init_f32(&mat_invGden, 6, 6, coeff_invGden);
    // arm_mat_init_f32(&mat_H, 6, 1, coeff_H);
    // arm_mat_init_f32(&mat_E, 6, 1, coeff_E);
    // arm_mat_init_f32(&mat_GE, 7, 1, coeff_GE);
    // arm_mat_init_f32(&mat_P, 7, 7, (float32_t *)coeff_P);
    // arm_mat_init_f32(&mat_GtransC, 7, 7, coeff_GtransC);
    // arm_mat_init_f32(&mat_GtransCbarP, 7, 7, coeff_GtransCbarP);

    InitializeKalmanFilter();

    roll_filt = 0;
    pitch_filt = 0;
    yaw_filt = 0;
    kappa = 0.8f;
}

// void TestMatrix()
// {
//     arm_matrix_instance_f32 mat_A;
//     arm_matrix_instance_f32 mat_trA;
//     arm_matrix_instance_f32 mat_B;
//     arm_matrix_instance_f32 mat_trAB;
//     arm_matrix_instance_f32 mat_test1;
//     arm_matrix_instance_f32 mat_test2;
//     float test_coeffA[4];
//     float test_coefftrA[4];
//     float test_coeffB[4];
//     float test_coefftrAB[4];
//     float test_coeff1[4] = {1.0, 2.0,
//                             3.0, 4.0};
//     float test_coeff2[4] = {10.0, 20.0,
//                             30.0, 40.0};
//     arm_mat_init_f32(&mat_A, 2, 2, test_coeffA);
//     arm_mat_init_f32(&mat_trA, 2, 2, test_coefftrA);
//     arm_mat_init_f32(&mat_B, 2, 2, test_coeffB);
//     arm_mat_init_f32(&mat_trAB, 2, 2, test_coefftrAB);
//     arm_mat_init_f32(&mat_test1, 2, 2, test_coeff1);
//     arm_mat_init_f32(&mat_test2, 2, 2, test_coeff2);

//     arm_mat_add_f32(&mat_test1, &mat_test2, &mat_A);
//     arm_mat_mult_f32(&mat_test1, &mat_test2, &mat_B);
//     arm_mat_trans_f32(&mat_A, &mat_trA);
//     arm_mat_mult_f32(&mat_trA, &mat_B, &mat_trAB);

//     printf("A = \n%3f, %3f\n%3f, %3f\n", test_coeffA[0], test_coeffA[1], test_coeffA[2], test_coeffA[3]);
//     printf("B = \n%3f, %3f\n%3f, %3f\n", test_coeffB[0], test_coeffB[1], test_coeffB[2], test_coeffB[3]);
//     printf("transA = \n%3f, %3f\n%3f, %3f\n", test_coefftrA[0], test_coefftrA[1], test_coefftrA[2], test_coefftrA[3]);
//     printf("transA*B = \n%3f, %3f\n%3f, %3f\n", test_coefftrAB[0], test_coefftrAB[1], test_coefftrAB[2], test_coefftrAB[3]);
// }

void UpdateAHRS(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs)
{
    // UpdateMadgwickFilter(acc, gyro, mag, ahrs);
    // UpdateMadgwickFilterIMU(acc, gyro, ahrs);
    // UpdateEKF(acc, gyro, mag, ahrs);
    // UpdateKF(acc, gyro, ahrs);
    UpdateComplimentaryFilter(acc, gyro, mag, ahrs);
}

void UpdateMadgwickFilter(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs)
{
    float recipNorm;
    float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    Quaternion dq;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mag->x == 0.0f) && (mag->y == 0.0f) && (mag->z == 0.0f))
    {
        UpdateMadgwickFilterIMU(acc, gyro, ahrs);
        return;
    }

    ahrs->gx = gyro->x;
    ahrs->gy = gyro->y;
    ahrs->gz = gyro->z;

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

        CalcNabla_fg(acc, ahrs);
        s0 = coeff_nabla_fg[0];
        s1 = coeff_nabla_fg[1];
        s2 = coeff_nabla_fg[2];
        s3 = coeff_nabla_fg[3];
        CalcNabla_fb(mag, ahrs);
        s0 += coeff_nabla_fb[0];
        s1 += coeff_nabla_fb[1];
        s2 += coeff_nabla_fb[2];
        s3 += coeff_nabla_fb[3];
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // compute and remove the gyroscope baises
        ahrs->gx_offset += 2.0f * (ahrs->q.q0 * s1 - ahrs->q.q1 * s0 - ahrs->q.q2 * s3 + ahrs->q.q3 * s2) * dt;
        ahrs->gy_offset += 2.0f * (ahrs->q.q0 * s2 + ahrs->q.q1 * s3 - ahrs->q.q2 * s0 - ahrs->q.q3 * s1) * dt;
        ahrs->gz_offset += 2.0f * (ahrs->q.q0 * s3 - ahrs->q.q1 * s2 + ahrs->q.q2 * s1 - ahrs->q.q3 * s0) * dt;
        // ahrs->gx_offset += 2.0f * (ahrs->q.q0 * s1 + ahrs->q.q1 * s0 + ahrs->q.q2 * s3 - ahrs->q.q3 * s2) * dt;
        // ahrs->gy_offset += 2.0f * (ahrs->q.q0 * s2 - ahrs->q.q1 * s3 + ahrs->q.q2 * s0 + ahrs->q.q3 * s1) * dt;
        // ahrs->gz_offset += 2.0f * (ahrs->q.q0 * s3 + ahrs->q.q1 * s2 - ahrs->q.q2 * s1 + ahrs->q.q3 * s0) * dt;

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
    }

    // Integrate rate of change of quaternion to yield quaternion
    ahrs->q.q0 += dq.q0 * dt;
    ahrs->q.q1 += dq.q1 * dt;
    ahrs->q.q2 += dq.q2 * dt;
    ahrs->q.q3 += dq.q3 * dt;
    QuaternionNorm(&(ahrs->q));

    Convertq2Euler(ahrs);
}

void UpdateMadgwickFilterIMU(AxesRaw *acc, AxesRaw *gyro, AHRS_State *ahrs)
{
    float recipNorm;
    float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    Quaternion dq;

    // Rate of change of quaternion from gyroscope
    dq.q0 = 0.5f * (-ahrs->q.q1 * gyro->x - ahrs->q.q2 * gyro->y - ahrs->q.q3 * gyro->z);
    dq.q1 = 0.5f * (ahrs->q.q0 * gyro->x + ahrs->q.q2 * gyro->z - ahrs->q.q3 * gyro->y);
    dq.q2 = 0.5f * (ahrs->q.q0 * gyro->y - ahrs->q.q1 * gyro->z + ahrs->q.q3 * gyro->x);
    dq.q3 = 0.5f * (ahrs->q.q0 * gyro->z + ahrs->q.q1 * gyro->y - ahrs->q.q2 * gyro->x);

    ahrs->gx = gyro->x;
    ahrs->gy = gyro->y;
    ahrs->gz = gyro->z;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((acc->x == 0.0f) && (acc->y == 0.0f) && (acc->z == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
        acc->x *= recipNorm;
        acc->y *= recipNorm;
        acc->z *= recipNorm;

        CalcNabla_fg(acc, ahrs);
        s0 = coeff_nabla_fg[0];
        s1 = coeff_nabla_fg[1];
        s2 = coeff_nabla_fg[2];
        s3 = coeff_nabla_fg[3];
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

    Convertq2Euler(ahrs);
}

void CalcNabla_fg(AxesRaw *acc, AHRS_State *ahrs)
{
    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0f * ahrs->q.q0;
    float _2q1 = 2.0f * ahrs->q.q1;
    float _2q2 = 2.0f * ahrs->q.q2;
    float _2q3 = 2.0f * ahrs->q.q3;
    float _4q1 = 4.0f * ahrs->q.q1;
    float _4q2 = 4.0f * ahrs->q.q2;
    float q0q1 = ahrs->q.q0 * ahrs->q.q1;
    float q0q2 = ahrs->q.q0 * ahrs->q.q2;
    float q1q1 = ahrs->q.q1 * ahrs->q.q1;
    float q1q3 = ahrs->q.q1 * ahrs->q.q3;
    float q2q2 = ahrs->q.q2 * ahrs->q.q2;
    float q2q3 = ahrs->q.q2 * ahrs->q.q3;

    // Gradient decent algorithm corrective step
    float coeff_fg[3] = {-2.0f * (q1q3 - q0q2) - acc->x,
                         -2.0f * (q0q1 + q2q3) - acc->y,
                         -2.0f * (0.5f - q1q1 - q2q2) - acc->z};
    arm_mat_init_f32(&mat_fg, 3, 1, coeff_fg);

    float coeff_transJg[12] = {_2q2, -_2q1, 0,
                               -_2q3, -_2q0, _4q1,
                               _2q0, -_2q3, _4q2,
                               -_2q1, -_2q2, 0};
    arm_mat_init_f32(&mat_transJg, 4, 3, coeff_transJg);

    arm_mat_mult_f32(&mat_transJg, &mat_fg, &mat_nabla_fg);
}

void CalcNabla_fb(AxesRaw *mag, AHRS_State *ahrs)
{
    // Auxiliary variables to avoid repeated arithmetic
    // float _2q0 = 2.0f * ahrs->q.q0;
    // float _2q1 = 2.0f * ahrs->q.q1;
    // float _2q2 = 2.0f * ahrs->q.q2;
    // float _2q3 = 2.0f * ahrs->q.q3;
    // float _2q0q2 = 2.0f * ahrs->q.q0 * ahrs->q.q2;
    // float _2q2q3 = 2.0f * ahrs->q.q2 * ahrs->q.q3;
    // float q0q0 = ahrs->q.q0 * ahrs->q.q0;
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
    // hx = mag->x * q0q0 - _2q0my * ahrs->q.q3 + _2q0mz * ahrs->q.q2 + mag->x * q1q1 + _2q1 * mag->y * ahrs->q.q2 + _2q1 * mag->z * ahrs->q.q3 - mag->x * q2q2 - mag->x * q3q3;
    // hy = _2q0mx * ahrs->q.q3 + mag->y * q0q0 - _2q0mz * ahrs->q.q1 + _2q1mx * ahrs->q.q2 - mag->y * q1q1 + mag->y * q2q2 + _2q2 * mag->z * ahrs->q.q3 - mag->y * q3q3;
    // float _2bx = Sqrt(hx * hx + hy * hy);
    // float _2bz = -_2q0mx * ahrs->q.q2 + _2q0my * ahrs->q.q1 + mag->z * q0q0 + _2q1mx * ahrs->q.q3 - mag->z * q1q1 + _2q2 * mag->y * ahrs->q.q3 - mag->z * q2q2 + mag->z * q3q3;
    // float _4bx = 2.0f * _2bx;
    // float _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    // s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q1 * (2.0f * q0q1 + _2q2q3 - acc->y) - _2bz * ahrs->q.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (-_2bx * ahrs->q.q3 + _2bz * ahrs->q.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + _2bx * ahrs->q.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);
    // s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q0 * (2.0f * q0q1 + _2q2q3 - acc->y) - 4.0f * ahrs->q.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - acc->z) + _2bz * ahrs->q.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (_2bx * ahrs->q.q2 + _2bz * ahrs->q.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + (_2bx * ahrs->q.q3 - _4bz * ahrs->q.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);
    // s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q3 * (2.0f * q0q1 + _2q2q3 - acc->y) - 4.0f * ahrs->q.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - acc->z) + (-_4bx * ahrs->q.q2 - _2bz * ahrs->q.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (_2bx * ahrs->q.q1 + _2bz * ahrs->q.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + (_2bx * ahrs->q.q0 - _4bz * ahrs->q.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);
    // s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - acc->x) + _2q2 * (2.0f * q0q1 + _2q2q3 - acc->y) + (-_4bx * ahrs->q.q3 + _2bz * ahrs->q.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mag->x) + (-_2bx * ahrs->q.q0 + _2bz * ahrs->q.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - mag->y) + _2bx * ahrs->q.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mag->z);

    float coeff_fb[3];
    coeff_fb[0] = _2bx * (0.5f - q2q2 - q3q3) + _2by * (q0q3 + q1q2) + _2bz * (q1q3 - q0q2) - mag->x;
    coeff_fb[1] = _2bx * (q1q2 - q0q3) + _2by * (0.5f - q1q1 - q3q3) + _2bz * (q0q1 + q2q3) - mag->y;
    coeff_fb[2] = _2bx * (q0q2 + q1q3) + _2by * (q2q3 - q0q1) + _2bz * (0.5f - q1q1 - q2q2) - mag->z;
    arm_mat_init_f32(&mat_fb, 3, 1, coeff_fb);

    // Auxiliary variables to avoid repeated arithmetic
    float _2bxq0 = _2bx * ahrs->q.q0;
    float _2bxq1 = _2bx * ahrs->q.q1;
    float _2bxq2 = _2bx * ahrs->q.q2;
    float _2bxq3 = _2bx * ahrs->q.q3;
    float _2byq0 = _2by * ahrs->q.q0;
    float _2byq1 = _2by * ahrs->q.q1;
    float _2byq2 = _2by * ahrs->q.q2;
    float _2byq3 = _2by * ahrs->q.q3;
    float _2bzq0 = _2bz * ahrs->q.q0;
    float _2bzq1 = _2bz * ahrs->q.q1;
    float _2bzq2 = _2bz * ahrs->q.q2;
    float _2bzq3 = _2bz * ahrs->q.q3;

    float coeff_transJb[12];
    coeff_transJb[0] = _2byq3 - _2bzq2;
    coeff_transJb[1] = -_2bxq3 + _2bzq1;
    coeff_transJb[2] = _2bxq2 - _2byq1;
    coeff_transJb[3] = _2byq2 + _2bzq3;
    coeff_transJb[4] = _2bxq2 - 2.0f * _2byq1 + _2bzq0;
    coeff_transJb[5] = _2bxq3 - _2byq0 - 2.0f * _2bzq1;
    coeff_transJb[6] = -2.0f * _2bxq2 + _2byq1 - _2bzq0;
    coeff_transJb[7] = _2bxq1 + _2bzq3;
    coeff_transJb[8] = _2bxq0 + _2byq3 - 2.0f * _2bzq2;
    coeff_transJb[9] = -2.0f * _2bxq3 + _2byq0 + _2bzq1;
    coeff_transJb[10] = -_2bxq0 - 2.0f * _2byq3 + _2bzq2;
    coeff_transJb[11] = _2bxq1 + _2byq2;
    arm_mat_init_f32(&mat_transJb, 4, 3, coeff_transJb);

    arm_mat_mult_f32(&mat_transJb, &mat_fb, &mat_nabla_fb);
}

void UpdateEKF(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs)
{
    // Normalise accelerometer measurement
    float recipNorm = invSqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
    acc->x *= recipNorm;
    acc->y *= recipNorm;
    acc->z *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mag->x * mag->x + mag->y * mag->y + mag->z * mag->z);
    mag->x *= recipNorm;
    mag->y *= recipNorm;
    mag->z *= recipNorm;

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

    float coeff_q0 = ahrs->q.q0 * half_dt;
    float coeff_q1 = ahrs->q.q1 * half_dt;
    float coeff_q2 = ahrs->q.q2 * half_dt;
    float coeff_q3 = ahrs->q.q3 * half_dt;

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
    float _2q0 = 2.0f * ahrs->q.q0;
    float _2q1 = 2.0f * ahrs->q.q1;
    float _2q2 = 2.0f * ahrs->q.q2;
    float _2q3 = 2.0f * ahrs->q.q3;
    float _2bxq0 = _2bx * ahrs->q.q0;
    float _2bxq1 = _2bx * ahrs->q.q1;
    float _2bxq2 = _2bx * ahrs->q.q2;
    float _2bxq3 = _2bx * ahrs->q.q3;
    float _2byq0 = _2by * ahrs->q.q0;
    float _2byq1 = _2by * ahrs->q.q1;
    float _2byq2 = _2by * ahrs->q.q2;
    float _2byq3 = _2by * ahrs->q.q3;
    float _2bzq0 = _2bz * ahrs->q.q0;
    float _2bzq1 = _2bz * ahrs->q.q1;
    float _2bzq2 = _2bz * ahrs->q.q2;
    float _2bzq3 = _2bz * ahrs->q.q3;

    float coeff_transC[42] = {
        _2q2, -_2q3, _2q0, -_2q1, 0.0f, 0.0f, 0.0f,
        -_2q1, -_2q0, -_2q3, -_2q2, 0.0f, 0.0f, 0.0f,
        -_2q0, _2q1, _2q2, -_2q3, 0.0f, 0.0f, 0.0f,
        (_2bxq0 + _2byq3 - _2bzq2), (_2bxq1 + _2byq2 + _2bzq3), (-_2bxq2 + _2byq1 - _2bzq0), (-_2bxq3 + _2byq0 + _2bzq1), 0.0f, 0.0f, 0.0f,
        (-_2bxq3 + _2byq0 + _2bzq1), (_2bxq2 - _2byq1 + _2bzq0), (_2bxq1 + _2byq2 + _2bzq3), (-_2bxq0 - _2byq3 + _2bzq2), 0.0f, 0.0f, 0.0f,
        (_2bxq2 - _2byq1 + _2bzq0), (_2bxq3 - _2byq0 - _2bzq1), (_2bxq0 + _2byq3 - _2bzq2), (_2bxq1 + _2byq2 + _2bzq3), 0.0f, 0.0f, 0.0f};
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

    Convertq2Euler(ahrs);
}

void UpdateKF(AxesRaw *acc, AxesRaw *gyro, AHRS_State *ahrs)
{
    float pitch = atan2(acc->x, Sqrt(acc->y * acc->y + acc->z * acc->z));
    ahrs->euler.y = getAngle(pitch, gyro->y, dt);
    ahrs->gy = getRate();
}

void UpdateComplimentaryFilter(AxesRaw *acc, AxesRaw *gyro, AxesRaw *mag, AHRS_State *ahrs)
{
    // float roll = -atan2(acc->y, sqrt(acc->x * acc->x + acc->z * acc->z));
    float roll = -atan2(acc->y, -acc->z);
    float pitch = atan2(acc->x, Sqrt(acc->y * acc->y + acc->z * acc->z));
    float yaw = atan2((-mag->y * cos(roll) + mag->z * sin(roll)), (mag->x * cos(pitch) + mag->y * sin(roll) * sin(pitch) + mag->z * cos(roll) * sin(pitch)));

    float dot_roll = gyro->x + (gyro->y * sin(ahrs->euler.x) + gyro->z * cos(ahrs->euler.x)) * tan(ahrs->euler.y);
    float dot_pitch = gyro->y * cos(ahrs->euler.x) - gyro->z * sin(ahrs->euler.x);
    float dot_yaw = (gyro->y * sin(ahrs->euler.x) + gyro->z * cos(ahrs->euler.x)) / cos(ahrs->euler.y);

    roll_filt = kappa * roll + (1.0 - kappa) * (roll_filt + dot_roll * dt);
    pitch_filt = kappa * pitch + (1.0 - kappa) * (pitch_filt + dot_pitch * dt);
    yaw_filt = kappa * yaw + (1.0 - kappa) * (yaw_filt + dot_yaw * dt);

    ahrs->euler.x = roll_filt;
    ahrs->euler.y = pitch_filt;
    ahrs->euler.z = yaw_filt;

    // ConvertEuler2q(ahrs);
}

void Convertq2Euler(AHRS_State *ahrs)
{
    // ZYX Euler
    ahrs->euler.x = -atan2(2.0f * (-ahrs->q.q0 * ahrs->q.q1 + ahrs->q.q2 * ahrs->q.q3), ahrs->q.q0 * ahrs->q.q0 - ahrs->q.q1 * ahrs->q.q1 - ahrs->q.q2 * ahrs->q.q2 + ahrs->q.q3 * ahrs->q.q3);
    ahrs->euler.y = asin(2.0f * (ahrs->q.q0 * ahrs->q.q2 + ahrs->q.q1 * ahrs->q.q3));
    ahrs->euler.z = -atan2(2.0f * (ahrs->q.q1 * ahrs->q.q2 - ahrs->q.q0 * ahrs->q.q3), ahrs->q.q0 * ahrs->q.q0 + ahrs->q.q1 * ahrs->q.q1 - ahrs->q.q2 * ahrs->q.q2 - ahrs->q.q3 * ahrs->q.q3);

    // XYZ Euler
    // ahrs->euler.x = atan2(2.0f * (ahrs->q.q0 * ahrs->q.q1 + ahrs->q.q2 * ahrs->q.q3), ahrs->q.q0 * ahrs->q.q0 - ahrs->q.q1 * ahrs->q.q1 - ahrs->q.q2 * ahrs->q.q2 + ahrs->q.q3 * ahrs->q.q3);
    // ahrs->euler.y = asin(2.0f * (ahrs->q.q0 * ahrs->q.q2 - ahrs->q.q1 * ahrs->q.q3));
    // ahrs->euler.z = atan2(2.0f * (ahrs->q.q1 * ahrs->q.q2 + ahrs->q.q0 * ahrs->q.q3), ahrs->q.q0 * ahrs->q.q0 + ahrs->q.q1 * ahrs->q.q1 - ahrs->q.q2 * ahrs->q.q2 - ahrs->q.q3 * ahrs->q.q3);
}

void ConvertEuler2q(AHRS_State *ahrs)
{
    // auxiliary variables to reduce number of repeated operations
    float sin_roll = sin(ahrs->euler.x * 0.5f);
    float cos_roll = cos(ahrs->euler.x * 0.5f);
    float sin_pitch = sin(ahrs->euler.y * 0.5f);
    float cos_pitch = cos(ahrs->euler.y * 0.5f);
    float sin_yaw = sin(ahrs->euler.z * 0.5f);
    float cos_yaw = cos(ahrs->euler.z * 0.5f);

    // ZYX Euler
    ahrs->q.q0 = -sin_roll * sin_pitch * sin_yaw + cos_roll * cos_pitch * cos_yaw;
    ahrs->q.q1 = cos_roll * sin_pitch * sin_yaw + sin_roll * cos_pitch * cos_yaw;
    ahrs->q.q2 = -sin_roll * cos_pitch * sin_yaw + cos_roll * sin_pitch * cos_yaw;
    ahrs->q.q3 = cos_roll * cos_pitch * sin_yaw + sin_roll * sin_pitch * cos_yaw;

    // XYZ Euler
    // ahrs->q.q0 = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_pitch * cos_yaw;
    // ahrs->q.q1 = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
    // ahrs->q.q2 = sin_roll * cos_pitch * sin_yaw + cos_roll * sin_pitch * cos_yaw;
    // ahrs->q.q3 = -sin_roll * sin_pitch * cos_yaw + cos_roll * cos_pitch * sin_yaw;
}