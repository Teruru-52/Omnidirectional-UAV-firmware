#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "main.h"
#include "basic_math.h"

// Type define for quaternion
typedef struct
{
    float q0, q1, q2, q3;
} Quaternion;

// Type define for Euler angle
typedef struct
{
    float roll, pitch, yaw;
} EulerAngle;

void QuaternionNorm(Quaternion *q);
void QuaternionMult(Quaternion *qa, Quaternion *qb, Quaternion *qo);
void QuaternionRotation(Quaternion *qr, Quaternion *qv, Quaternion *qo);
void QuaternionConj(Quaternion *qa, Quaternion *qo);
void Quaternion2Euler(Quaternion *qr, EulerAngle *ea);

#define MAX_RAD 1.5

#endif /* _QUATERNION_H_ */
