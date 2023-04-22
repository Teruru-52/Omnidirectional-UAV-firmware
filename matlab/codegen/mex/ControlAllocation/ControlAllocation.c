/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ControlAllocation.c
 *
 * Code generation for function 'ControlAllocation'
 *
 */

/* Include files */
#include "ControlAllocation.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Variable Definitions */
static emlrtMCInfo emlrtMCI = {
    17,                  /* lineNo */
    1,                   /* colNo */
    "ControlAllocation", /* fName */
    "/home/reiji/Documents/Robots/Omnidirectional_uav/matlab/"
    "ControlAllocation.m" /* pName */
};

static emlrtMCInfo b_emlrtMCI = {
    18,                  /* lineNo */
    5,                   /* colNo */
    "ControlAllocation", /* fName */
    "/home/reiji/Documents/Robots/Omnidirectional_uav/matlab/"
    "ControlAllocation.m" /* pName */
};

static emlrtRSInfo emlrtRSI = {
    17,                  /* lineNo */
    "ControlAllocation", /* fcnName */
    "/home/reiji/Documents/Robots/Omnidirectional_uav/matlab/"
    "ControlAllocation.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    18,                  /* lineNo */
    "ControlAllocation", /* fcnName */
    "/home/reiji/Documents/Robots/Omnidirectional_uav/matlab/"
    "ControlAllocation.m" /* pathName */
};

/* Function Declarations */
static const mxArray *coder_internal_mxSubscript(const emlrtStack *sp,
                                                 const mxArray *m1,
                                                 const mxArray *m2,
                                                 emlrtMCInfo *location);

static void linprog(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    const mxArray *m2, const mxArray *m3, const mxArray *m4,
                    const mxArray *m5, const mxArray *m6, emlrtMCInfo *location,
                    const mxArray **r, const mxArray **r1, const mxArray **r2,
                    const mxArray **r3);

/* Function Definitions */
static const mxArray *coder_internal_mxSubscript(const emlrtStack *sp,
                                                 const mxArray *m1,
                                                 const mxArray *m2,
                                                 emlrtMCInfo *location)
{
  const mxArray *pArrays[2];
  const mxArray *m;
  pArrays[0] = m1;
  pArrays[1] = m2;
  return emlrtCallMATLABR2012b((emlrtConstCTX)sp, 1, &m, 2, &pArrays[0],
                               "coder.internal.mxSubscript", true, location);
}

static void linprog(const emlrtStack *sp, const mxArray *m, const mxArray *m1,
                    const mxArray *m2, const mxArray *m3, const mxArray *m4,
                    const mxArray *m5, const mxArray *m6, emlrtMCInfo *location,
                    const mxArray **r, const mxArray **r1, const mxArray **r2,
                    const mxArray **r3)
{
  const mxArray *pArrays[7];
  const mxArray *mv[4];
  pArrays[0] = m;
  pArrays[1] = m1;
  pArrays[2] = m2;
  pArrays[3] = m3;
  pArrays[4] = m4;
  pArrays[5] = m5;
  pArrays[6] = m6;
  emlrtAssign(r, emlrtCallMATLABR2012b((emlrtConstCTX)sp, 4, &mv[0], 7,
                                       &pArrays[0], "linprog", true, location));
  emlrtAssign(r1, mv[1]);
  emlrtAssign(r2, mv[2]);
  emlrtAssign(r3, mv[3]);
}

void ControlAllocation(const emlrtStack *sp, real_T tau_x, real_T tau_y,
                       real_T tau_z, const mxArray **u, const mxArray **flag)
{
  static const real_T M[27] = {
      0.0624,  -0.2331, 0.1706, -0.2331, 0.0926,  0.0411, 0.0781,
      0.0624,  -0.1139, 0.0926, 0.0781,  -0.0979, 0.1192, -0.0213,
      -0.1706, -0.0213, 0.0358, -0.0411, -0.1337, 0.1192, 0.1139,
      0.0358,  -0.1337, 0.0979, 0.0,     0.0,     0.0};
  static const int32_T iv1[2] = {1, 9};
  static const int32_T iv2[2] = {17, 9};
  static const int32_T iv3[2] = {3, 9};
  static const int32_T iv4[2] = {0, 0};
  static const int32_T iv5[2] = {0, 0};
  static const int32_T iv6[2] = {1, 8};
  static const int32_T i1 = 17;
  static const int32_T i2 = 3;
  static const int8_T f[9] = {0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const int8_T iv[9] = {0, 0, 0, 0, 0, 0, 0, 0, -1};
  emlrtStack st;
  const mxArray *a__1;
  const mxArray *a__2;
  const mxArray *b_flag;
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *fr;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *m;
  const mxArray *y;
  real_T A[153];
  real_T *pData;
  int32_T b_i;
  int32_T i;
  int32_T k;
  int8_T A_tmp[64];
  st.prev = sp;
  st.tls = sp->tls;
  *flag = NULL;
  *u = NULL;
  a__1 = NULL;
  a__2 = NULL;
  fr = NULL;
  b_flag = NULL;
  memset(&A_tmp[0], 0, 64U * sizeof(int8_T));
  for (k = 0; k < 8; k++) {
    A_tmp[k + (k << 3)] = 1;
  }
  for (i = 0; i < 8; i++) {
    for (k = 0; k < 8; k++) {
      A[k + 17 * i] = -(real_T)A_tmp[k + (i << 3)];
    }
    A[i + 136] = 0.0;
    for (k = 0; k < 8; k++) {
      A[(k + 17 * i) + 8] = A_tmp[k + (i << 3)];
    }
    A[i + 144] = -0.5;
  }
  for (i = 0; i < 9; i++) {
    A[17 * i + 16] = iv[i];
  }
  y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv1[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (k = 0; k < 9; k++) {
    pData[k] = f[k];
  }
  emlrtAssign(&y, m);
  b_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv2[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (k = 0; k < 9; k++) {
    for (b_i = 0; b_i < 17; b_i++) {
      pData[i + b_i] = A[b_i + 17 * k];
    }
    i += 17;
  }
  emlrtAssign(&b_y, m);
  c_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (k = 0; k < 17; k++) {
    pData[k] = 0.0;
  }
  emlrtAssign(&c_y, m);
  d_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv3[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (k = 0; k < 9; k++) {
    pData[i] = M[3 * k];
    pData[i + 1] = M[3 * k + 1];
    pData[i + 2] = M[3 * k + 2];
    i += 3;
  }
  emlrtAssign(&d_y, m);
  e_y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = tau_x;
  pData[1] = tau_y;
  pData[2] = tau_z;
  emlrtAssign(&e_y, m);
  f_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv4[0], mxDOUBLE_CLASS, mxREAL);
  emlrtAssign(&f_y, m);
  g_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv5[0], mxDOUBLE_CLASS, mxREAL);
  emlrtAssign(&g_y, m);
  st.site = &emlrtRSI;
  linprog(&st, y, b_y, c_y, d_y, e_y, f_y, g_y, &emlrtMCI, &fr, &a__1, &b_flag,
          &a__2);
  emlrtAssign(flag, emlrtAlias(b_flag));
  h_y = NULL;
  m = emlrtCreateNumericArray(2, (const void *)&iv6[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  for (k = 0; k < 8; k++) {
    pData[k] = (real_T)k + 1.0;
  }
  emlrtAssign(&h_y, m);
  st.site = &b_emlrtRSI;
  emlrtAssign(
      u, coder_internal_mxSubscript(&st, emlrtAlias(fr), h_y, &b_emlrtMCI));
  emlrtDestroyArray(&a__1);
  emlrtDestroyArray(&a__2);
  emlrtDestroyArray(&fr);
  emlrtDestroyArray(&b_flag);
}

/* End of code generation (ControlAllocation.c) */
