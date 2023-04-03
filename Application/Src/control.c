/*
 *  control.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "control.h"

Quaternion q_des = {1.0, 0.0, 0.0, 0.0};
Quaternion dq_des = {0.0, 0.0, 0.0, 0.0};
Quaternion q_err, omega_ff;
AxesRaw omega_des;
const float Jxx = 0.0, Jyy = 0.0, Jzz = 0.0;
const float tau_att = 0.001;
const float tau_omega = 0.001;
float coeff_tau_att, coeff_tau_omega;
float coeff_Jxx, coeff_Jyy, coeff_Jzz;

void InitializeController()
{
    coeff_tau_att = 2.0f / tau_att;
    coeff_tau_omega = 1.0f / tau_omega;
    coeff_Jxx = coeff_tau_omega * Jxx;
    coeff_Jyy = coeff_tau_omega * Jyy;
    coeff_Jzz = coeff_tau_omega * Jzz;

    float coeff_pinvM[24] = {
        0.3235, -1.2074, 0.8839,
        -1.2074, -0.3235, -0.8839,
        1.2074, 0.3235, -0.8839,
        -0.3235, 1.2074, 0.8839,
        0.3235, -1.2074, -0.8839,
        -1.2074, -0.3235, 0.8839,
        1.2074, 0.3235, 0.8839,
        -0.3235, 1.2074, -0.8839};

    arm_mat_init_f32(&mat_pinvM, 8, 3, coeff_pinvM);
    arm_mat_init_f32(&mat_Tdes, 3, 1, coeff_Tdes);
    arm_mat_init_f32(&mat_Fprop, 8, 1, coeff_Fprop);
}

void UpdateControl(AHRS_State *ahrs, MotorInput *motor_input, float *bat_vol)
{
    Quaternion q_conj, q_des_conj, q_err_conj, q_tmp1, q_tmp2;

    QuaternionConj(&(ahrs->q), &q_conj);
    QuaternionMult(&q_conj, &q_des, &q_err);

    QuaternionConj(&q_des, &q_des_conj);
    QuaternionMult(&q_err, &q_des_conj, &q_tmp1);
    QuaternionMult(&q_tmp1, &dq_des, &q_tmp2);
    QuaternionConj(&q_err, &q_err_conj);
    QuaternionMult(&q_tmp2, &q_err_conj, &omega_ff);

    omega_des.x = coeff_tau_att * Sign(q_err.q0) * q_err.q1 + 2.0f * omega_ff.q1;
    omega_des.y = coeff_tau_att * Sign(q_err.q0) * q_err.q2 + 2.0f * omega_ff.q2;
    omega_des.z = coeff_tau_att * Sign(q_err.q0) * q_err.q3 + 2.0f * omega_ff.q3;

    coeff_Tdes[0] = coeff_Jxx * (omega_des.x - ahrs->gx);
    coeff_Tdes[1] = coeff_Jyy * (omega_des.y - ahrs->gy);
    coeff_Tdes[2] = coeff_Jzz * (omega_des.z - ahrs->gz);
    arm_mat_init_f32(&mat_Tdes, 3, 1, coeff_Tdes);

    arm_mat_mult_f32(&mat_pinvM, &mat_Tdes, &mat_Fprop);
    CalcMotorInput(motor_input, bat_vol);
}

void CalcMotorInput(MotorInput *motor_input, float *bat_vol)
{
    // feed-foward control
    motor_input->input1 = coeff_Fprop[0];
    motor_input->input2 = coeff_Fprop[1];
    motor_input->input3 = coeff_Fprop[2];
    motor_input->input4 = coeff_Fprop[3];
    motor_input->input5 = coeff_Fprop[4];
    motor_input->input6 = coeff_Fprop[5];
    motor_input->input7 = coeff_Fprop[6];
    motor_input->input8 = coeff_Fprop[7];

    motor_input->input1 = Voltage2Duty(motor_input->input1, bat_vol);
    motor_input->input2 = Voltage2Duty(motor_input->input2, bat_vol);
    motor_input->input3 = Voltage2Duty(motor_input->input3, bat_vol);
    motor_input->input4 = Voltage2Duty(motor_input->input4, bat_vol);
    motor_input->input5 = Voltage2Duty(motor_input->input5, bat_vol);
    motor_input->input6 = Voltage2Duty(motor_input->input6, bat_vol);
    motor_input->input7 = Voltage2Duty(motor_input->input7, bat_vol);
    motor_input->input8 = Voltage2Duty(motor_input->input8, bat_vol);
}

float Voltage2Duty(float voltage, float *bat_vol)
{
    return (float)(MOTOR_MAX_PWM_VALUE)*voltage / (*bat_vol);
}