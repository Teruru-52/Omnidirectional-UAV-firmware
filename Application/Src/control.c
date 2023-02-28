/*
 *  control.c
 *
 *  Created on: February 23rd, 2023
 *      Author: Reiji Terunuma
 */

#include "control.h"

Quaternion q_des = {1.0, 0.0, 0.0, 0.0};
Quaternion dq_des = {0.0, 0.0, 0.0, 0.0};
Quaternion q_err, q_tmp;
AxesRaw omega_des, omega_ff, torque_des;
const float Jxx = 0.0, Jyy = 0.0, Jzz = 0.0;

void UpdateControl(AHRS_State *ahrs, MotorControl *motor)
{
    QuaternionConj(&(ahrs->q), &q_tmp);
    QuaternionMult(&q_tmp, &q_des, &q_err);
    // QuaternionMult(&q_err, &q_des, &q_tmp);

    omega_des.x = Sign(q_err.q0) * q_err.q1 + omega_ff.x;
    omega_des.y = Sign(q_err.q0) * q_err.q2 + omega_ff.y;
    omega_des.z = Sign(q_err.q0) * q_err.q3 + omega_ff.z;

    torque_des.x = Jxx * (omega_des.x - ahrs->gx);
    torque_des.y = Jyy * (omega_des.y - ahrs->gy);
    torque_des.z = Jzz * (omega_des.z - ahrs->gz);
}
