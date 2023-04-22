/*
 *  control.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "control.h"

Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0

Quaternion q_des_edge;
Quaternion q_des_vertex;
Quaternion dq_des = {0.0, 0.0, 0.0, 0.0};
Quaternion q_err, omega_ff;
AxesRaw omega_des;
const float J = 0.0017f;
const float tau_att = 0.5f;
const float tau_omega = 0.5f;
const float kappa_f = 8e-7f;
const float param_rps2voltage[5] = {2.1967e-09, -1.1731e-6, 2.3771e-04, -0.0136, 0.5331};
const float rps_max = 248.333f;
const float err_threshold = 0.05;
float coeff_tau_att, coeff_tau_omega, coeff_kappa_f;
float coeff_J;

// around the center of gravity
// const float32_t coeff_pinvM[24] = {
//     1.2579f, -4.6945f, 3.4366f,
//     -4.6945f, -1.2579f, -3.4366f,
//     4.6945f, 1.2579f, -3.4366f,
//     -1.2579f, 4.6945f, 3.4366f,
//     1.2579f, -4.6945f, -3.4366f,
//     -4.6945f, -1.2579f, 3.4366f,
//     4.6945f, 1.2579f, 3.4366f,
//     -1.2579f, 4.6945f, -3.4366f};

// around the center of rotation
const float32_t coeff_pinvM[24] = {
    0.7109f, -2.6530f, 1.9421f,
    -5.2177f, -1.7494f, -2.4220f,
    1.8139f, 1.3981f, -0.6962f,
    2.6929f, 3.0042f, 1.1761f,
    -0.6080f, -2.4456f, -3.8196f,
    -1.2553f, -0.4305f, -0.8300f,
    1.6066f, 3.8342f, 3.9482f,
    0.2567f, -0.9582f, 0.7014f};

void InitializeController()
{
    // const float angle_offset = M_PI / 37.5f;
    const float angle_offset = 0;
    const float theta_des = -M_PI / 4.0f + angle_offset;
    // inverted at the edge
    q_des_edge.q0 = cos(theta_des / 2.0f);
    q_des_edge.q1 = 0.0f;
    q_des_edge.q2 = sin(theta_des / 2.0f);
    q_des_edge.q3 = 0.0f;
    // inverted at the vertex
    const float phi_des = atan(1.0f / sqrt(2.0f)) + angle_offset;
    q_des_vertex.q0 = cos(phi_des / 2.0f) * cos(theta_des / 2.0f);
    q_des_vertex.q1 = sin(phi_des / 2.0f) * cos(theta_des / 2.0f);
    q_des_vertex.q2 = cos(phi_des / 2.0f) * sin(theta_des / 2.0f);
    q_des_vertex.q3 = -sin(phi_des / 2.0f) * sin(theta_des / 2.0f);
    // printf("q_des_vertex = %.3f,\t%.3f,\t%.3f,\t%.3f\t\r\n", q_des_vertex.q0, q_des_vertex.q1, q_des_vertex.q2, q_des_vertex.q3);

    coeff_tau_att = 2.0f / tau_att;
    coeff_tau_omega = 1.0f / tau_omega;
    coeff_kappa_f = 1.0f / kappa_f;
    coeff_J = coeff_tau_omega * J;

    arm_mat_init_f32(&mat_pinvM, 8, 3, (float32_t *)coeff_pinvM);
    arm_mat_init_f32(&mat_Tdes, 3, 1, coeff_Tdes);
    arm_mat_init_f32(&mat_Fprop, 8, 1, coeff_Fprop);

    set_defaults();
    setup_indexing();
    load_default_data();
}

void load_default_data(void)
{
    // params.M[0] = -0.7887;
    // params.M[1] = 0.2113;
    // params.M[2] = -0.2113;
    // params.M[3] = 0.7887;
    // params.M[4] = 0.7887;
    // params.M[5] = -0.2113;
    // params.M[6] = 0.2113;
    // params.M[7] = -0.7887;
    // params.M[8] = 0.2113;
    // params.M[9] = 0.7887;
    // params.M[10] = -0.7887;
    // params.M[11] = -0.2113;
    // params.M[12] = -0.2113;
    // params.M[13] = -0.7887;
    // params.M[14] = 0.7887;
    // params.M[15] = 0.2113;
    // params.M[16] = 0.5774;
    // params.M[17] = -0.5774;
    // params.M[18] = -0.5774;
    // params.M[19] = 0.5774;
    // params.M[20] = 0.5774;
    // params.M[21] = -0.5774;
    // params.M[22] = -0.5774;
    // params.M[23] = 0.5774;
    // params.M[24] = 0.0624;
    // params.M[25] = -0.2331;
    // params.M[26] = 0.0781;
    // params.M[27] = 0.0926;
    // params.M[28] = 0.1192;
    // params.M[29] = -0.0213;
    // params.M[30] = -0.1337;
    // params.M[31] = 0.0358;
    // params.M[32] = -0.2331;
    // params.M[33] = 0.0926;
    // params.M[34] = 0.0624;
    // params.M[35] = 0.0781;
    // params.M[36] = -0.0213;
    // params.M[37] = 0.0358;
    // params.M[38] = 0.1192;
    // params.M[39] = -0.1337;
    // params.M[40] = 0.1706;
    // params.M[41] = 0.0411;
    // params.M[42] = -0.1139;
    // params.M[43] = -0.0979;
    // params.M[44] = -0.1706;
    // params.M[45] = -0.0411;
    // params.M[46] = 0.1139;
    // params.M[47] = 0.0979;
    // params.u[0] = 0;
    // params.u[1] = 0;
    // params.u[2] = 0;
    // params.u[3] = 0;
    // params.u[4] = 0;
    // params.u[5] = 0;
    // /* Make this a diagonal PSD matrix, even though it's not diagonal. */
    // params.Q[0] = 1.0;
    // params.Q[6] = 0;
    // params.Q[12] = 0;
    // params.Q[18] = 0;
    // params.Q[24] = 0;
    // params.Q[30] = 0;
    // params.Q[1] = 0;
    // params.Q[7] = 1.0;
    // params.Q[13] = 0;
    // params.Q[19] = 0;
    // params.Q[25] = 0;
    // params.Q[31] = 0;
    // params.Q[2] = 0;
    // params.Q[8] = 0;
    // params.Q[14] = 1.0;
    // params.Q[20] = 0;
    // params.Q[26] = 0;
    // params.Q[32] = 0;
    // params.Q[3] = 0;
    // params.Q[9] = 0;
    // params.Q[15] = 0;
    // params.Q[21] = 1.0;
    // params.Q[27] = 0;
    // params.Q[33] = 0;
    // params.Q[4] = 0;
    // params.Q[10] = 0;
    // params.Q[16] = 0;
    // params.Q[22] = 0;
    // params.Q[28] = 1.0;
    // params.Q[34] = 0;
    // params.Q[5] = 0;
    // params.Q[11] = 0;
    // params.Q[17] = 0;
    // params.Q[23] = 0;
    // params.Q[29] = 0;
    // params.Q[35] = 1.0;
    // params.fmin[0] = 0.0;
    // params.fmax[0] = 0.06;
    // f_max = kappa_f * rps_max * rps_max;

    params.M[0] = 0.0417;
    params.M[1] = -0.1556;
    params.M[2] = 0.1139;
    params.M[3] = -0.1556;
    params.M[4] = 0.0478;
    params.M[5] = 0.0084;
    params.M[6] = 0.0661;
    params.M[7] = 0.0417;
    params.M[8] = -0.0811;
    params.M[9] = 0.0478;
    params.M[10] = 0.0661;
    params.M[11] = -0.0411;
    params.M[12] = 0.0744;
    params.M[13] = -0.0333;
    params.M[14] = -0.1139;
    params.M[15] = -0.0333;
    params.M[16] = 0.0151;
    params.M[17] = -0.0084;
    params.M[18] = -0.0562;
    params.M[19] = 0.0744;
    params.M[20] = 0.0811;
    params.M[21] = 0.0151;
    params.M[22] = -0.0562;
    params.M[23] = 0.0411;
    params.tau[0] = 0;
    params.tau[1] = 0;
    params.tau[2] = 0;
    /* Make this a diagonal PSD matrix, even though it's not diagonal. */
    params.Q[0] = 1.0;
    params.Q[3] = 0;
    params.Q[6] = 0;
    params.Q[1] = 0;
    params.Q[4] = 1.0;
    params.Q[7] = 0;
    params.Q[2] = 0;
    params.Q[5] = 0;
    params.Q[8] = 1.0;
    params.fmin[0] = 0.0;
    params.fmax[0] = 0.049;
}

void TestControl(AHRS_State *ahrs)
{
    Quaternion q_subs, q_des;
    q_des = q_des_edge;
    q_des = q_des_vertex;
    q_subs.q0 = ahrs->q.q0 - q_des.q0;
    q_subs.q1 = ahrs->q.q1 - q_des.q1;
    q_subs.q2 = ahrs->q.q2 - q_des.q2;
    q_subs.q3 = ahrs->q.q3 - q_des.q3;

    float errorNorm = Sqrt(q_subs.q0 * q_subs.q0 + q_subs.q1 * q_subs.q1 + q_subs.q2 * q_subs.q2 + q_subs.q3 * q_subs.q3);
    if (errorNorm > err_threshold)
    {
        Write_GPIO(USER_LED2, 0);
    }
    else
    {
        Write_GPIO(USER_LED2, 1);
    }
}

void UpdateControl(AHRS_State *ahrs, MotorInput *motor_input, float *bat_vol)
{
    Quaternion q_subs_edge, q_subs_vertex, q_des, q_conj;
    // Quaternion q_des_conj, q_err_conj, q_tmp1, q_tmp2;

    q_subs_edge.q0 = ahrs->q.q0 - q_des_edge.q0;
    q_subs_edge.q1 = 0;
    // q_subs_edge.q1 = ahrs->q.q1 - q_des_edge.q1;
    q_subs_edge.q2 = ahrs->q.q2 - q_des_edge.q2;
    q_subs_edge.q3 = 0;
    // q_subs_edge.q3 = ahrs->q.q3 - q_des_edge.q3;

    q_subs_vertex.q0 = ahrs->q.q0 - q_des_vertex.q0;
    q_subs_vertex.q1 = ahrs->q.q1 - q_des_vertex.q1;
    q_subs_vertex.q2 = ahrs->q.q2 - q_des_vertex.q2;
    q_subs_vertex.q3 = ahrs->q.q3 - q_des_vertex.q3;

    float errorNorm_edge = Sqrt(q_subs_edge.q0 * q_subs_edge.q0 + q_subs_edge.q1 * q_subs_edge.q1 + q_subs_edge.q2 * q_subs_edge.q2 + q_subs_edge.q3 * q_subs_edge.q3);
    // float errorNorm_vertex = Sqrt(q_subs_vertex.q0 * q_subs_vertex.q0 + q_subs_vertex.q1 * q_subs_vertex.q1 + q_subs_vertex.q2 * q_subs_vertex.q2 + q_subs_vertex.q3 * q_subs_vertex.q3);
    // if (errorNorm_edge < err_threshold || errorNorm_vertex < err_threshold)
    if (errorNorm_edge < err_threshold)
    {
        q_des = q_des_edge;
        // q_des = q_des_vertex;

        Write_GPIO(USER_LED2, 1);

        QuaternionConj(&(ahrs->q), &q_conj);
        QuaternionMult(&q_conj, &q_des, &q_err);
        // printf("%.3f\t%.3f\t%.3f\t%.3f\t\r\n", q_err.q0, q_err.q1, q_err.q2, q_err.q3);

        // QuaternionConj(&q_des, &q_des_conj);
        // QuaternionMult(&q_err, &q_des_conj, &q_tmp1);
        // QuaternionMult(&q_tmp1, &dq_des, &q_tmp2);
        // QuaternionConj(&q_err, &q_err_conj);
        // QuaternionMult(&q_tmp2, &q_err_conj, &omega_ff);

        // omega_des.x = coeff_tau_att * Sign(q_err.q0) * q_err.q1 + omega_ff.q1;
        // omega_des.y = coeff_tau_att * Sign(q_err.q0) * q_err.q2 + omega_ff.q2;
        // omega_des.z = coeff_tau_att * Sign(q_err.q0) * q_err.q3 + omega_ff.q3;
        omega_des.x = coeff_tau_att * Sign(q_err.q0) * q_err.q1;
        omega_des.y = coeff_tau_att * Sign(q_err.q0) * q_err.q2;
        omega_des.z = coeff_tau_att * Sign(q_err.q0) * q_err.q3;
        // printf("%.3f\t%.3f\t%.3f\t\r\n", omega_des.x, omega_des.y, omega_des.z);

        coeff_Tdes[0] = coeff_J * (omega_des.x - ahrs->gx);
        coeff_Tdes[1] = coeff_J * (omega_des.y - ahrs->gy);
        coeff_Tdes[2] = coeff_J * (omega_des.z - ahrs->gz);
        // arm_mat_init_f32(&mat_Tdes, 3, 1, coeff_Tdes);
        // printf("%.3f\t%.3f\t%.3f\t\r\n", coeff_Tdes[0], coeff_Tdes[1], coeff_Tdes[2]);

        params.tau[0] = coeff_Tdes[0];
        params.tau[1] = coeff_Tdes[1];
        params.tau[2] = coeff_Tdes[2];
        // params.u[3] = coeff_Tdes[0];
        // params.u[4] = coeff_Tdes[1];
        // params.u[5] = coeff_Tdes[2];
        solve();
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            coeff_Fprop[i] = vars.x[i];
        }
        // printf("objv = %10.3e\n", work.optval);

        // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", coeff_Tdes[0], coeff_Tdes[1], coeff_Tdes[2],
        //        coeff_Fprop[0], coeff_Fprop[1], coeff_Fprop[2], coeff_Fprop[3], coeff_Fprop[4], coeff_Fprop[5], coeff_Fprop[6], coeff_Fprop[7], work.optval);

        // arm_mat_mult_f32(&mat_pinvM, &mat_Tdes, &mat_Fprop);
        // printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", coeff_Fprop[0], coeff_Fprop[1], coeff_Fprop[2], coeff_Fprop[3],
        //        coeff_Fprop[4], coeff_Fprop[5], coeff_Fprop[6], coeff_Fprop[7]);

        CalcMotorInput(motor_input, bat_vol);
        // printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", motor_input->inputs[0], motor_input->inputs[1], motor_input->inputs[2], motor_input->inputs[3],
        //        motor_input->inputs[4], motor_input->inputs[5], motor_input->inputs[6], motor_input->inputs[7]);
    }
    else if (errorNorm_edge > err_threshold)
    {
        Write_GPIO(USER_LED2, 0);
        for (int i = 0; i < MOTOR_NUM; i++)
        {
            motor_input->inputs[i] = 0;
        }
    }
}

void CalcMotorInput(MotorInput *motor_input, float *bat_vol)
{
    // calculate input velocity
    CalcInputVelocity(motor_input);

    // feed-foward control
    // calculate input voltage
    CalcInputVoltage(motor_input);

    // calculate input duty
    Voltage2Duty(motor_input, bat_vol);
}

void CalcInputVelocity(MotorInput *motor_input)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (coeff_Fprop[i] < 0)
        {
            motor_input->inputs[i] = 0.0;
        }
        else
        {
            motor_input->inputs[i] = Sqrt(coeff_kappa_f * coeff_Fprop[i]);
        }
    }
}

void CalcInputVoltage(MotorInput *motor_input)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        if (motor_input->inputs[i] < 0)
        {
            motor_input->inputs[i] = 0.0;
        }
        else
        {
            float rps = motor_input->inputs[i];
            motor_input->inputs[i] = param_rps2voltage[0] * (rps * rps * rps * rps) + param_rps2voltage[1] * (rps * rps * rps) + param_rps2voltage[2] * (rps * rps) + param_rps2voltage[3] * rps + param_rps2voltage[4];
        }
    }
}

void Voltage2Duty(MotorInput *motor_input, float *bat_vol)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        motor_input->inputs[i] = (float)(MOTOR_MAX_PWM_VALUE)*motor_input->inputs[i] / (*bat_vol);
    }
}