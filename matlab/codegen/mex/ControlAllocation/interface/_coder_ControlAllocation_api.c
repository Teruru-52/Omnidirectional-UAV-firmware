/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_ControlAllocation_api.c
 *
 * Code generation for function '_coder_ControlAllocation_api'
 *
 */

/* Include files */
#include "_coder_ControlAllocation_api.h"
#include "ControlAllocation.h"
#include "ControlAllocation_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *tau_x,
                               const char_T *identifier);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *tau_x,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(tau_x), &thisId);
  emlrtDestroyArray(&tau_x);
  return y;
}

void ControlAllocation_api(const mxArray *const prhs[3], int32_T nlhs,
                           const mxArray *plhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *flag;
  const mxArray *u;
  real_T tau_x;
  real_T tau_y;
  real_T tau_z;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  tau_x = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "tau_x");
  tau_y = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "tau_y");
  tau_z = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "tau_z");
  /* Invoke the target function */
  ControlAllocation(&st, tau_x, tau_y, tau_z, &u, &flag);
  /* Marshall function outputs */
  plhs[0] = u;
  if (nlhs > 1) {
    plhs[1] = flag;
  }
}

/* End of code generation (_coder_ControlAllocation_api.c) */
