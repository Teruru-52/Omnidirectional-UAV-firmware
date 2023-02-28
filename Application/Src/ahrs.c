/*
 *  ahrs.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "ahrs.h"

float dt;
float half_dt;
float ga;
float mn;
float md;
float double_ga;
float double_mn;
float double_md;

void InitializeEKF(AHRS_State *ahrs)
{
    ahrs->q.q0 = 1.0f;
    ahrs->q.q1 = 0.0f;
    ahrs->q.q2 = 0.0f;
    ahrs->q.q3 = 0.0f;
    ahrs->gx_offset = 0.0f;
    ahrs->gy_offset = 0.0f;
    ahrs->gz_offset = 0.0f;

    dt = 0.001f;
    half_dt = dt * 0.5f;
    ga = 9.81f;
    // mn = 0.0f;
    // md = 0.0f;
    double_ga = 2.0f * ga;
    double_mn = 2.0f * mn;
    double_md = 2.0f * md;

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
    coeff_gm[2] = -ga;
    coeff_gm[3] = mn;
    coeff_gm[4] = 0.0f;
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
    float coeff_gaq0 = double_ga * ahrs->q.q0;
    float coeff_gaq1 = double_ga * ahrs->q.q1;
    float coeff_gaq2 = double_ga * ahrs->q.q2;
    float coeff_gaq3 = double_ga * ahrs->q.q3;
    float coeff_mnq0 = double_mn * ahrs->q.q0;
    float coeff_mnq1 = double_mn * ahrs->q.q1;
    float coeff_mnq2 = double_mn * ahrs->q.q2;
    float coeff_mnq3 = double_mn * ahrs->q.q3;
    float coeff_mdq0 = double_md * ahrs->q.q0;
    float coeff_mdq1 = double_md * ahrs->q.q1;
    float coeff_mdq2 = double_md * ahrs->q.q2;
    float coeff_mdq3 = double_md * ahrs->q.q3;

    float coeff_transC[42] = {
        coeff_gaq2, -coeff_gaq3, coeff_gaq0, -coeff_gaq1, 0.0f, 0.0f, 0.0f,
        -coeff_gaq1, -coeff_gaq0, -coeff_gaq3, -coeff_gaq2, 0.0f, 0.0f, 0.0f,
        -coeff_gaq0, coeff_gaq1, coeff_gaq2, -coeff_gaq3, 0.0f, 0.0f, 0.0f,
        coeff_mnq0 - coeff_mdq2, coeff_mnq1 + coeff_mdq3, -coeff_mnq2 - coeff_mdq0, -coeff_mnq3 + coeff_mdq1, 0.0f, 0.0f, 0.0f,
        -coeff_mnq3 + coeff_mdq1, coeff_mnq2 + coeff_mdq0, coeff_mnq1 + coeff_mdq3, -coeff_mnq0 + coeff_mdq2, 0.0f, 0.0f, 0.0f,
        coeff_mnq2 + coeff_mdq0, coeff_mnq3 - coeff_mdq1, coeff_mnq0 - coeff_mdq2, coeff_mnq1 + coeff_mdq3, 0.0f, 0.0f, 0.0f};
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
}