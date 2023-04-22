/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ControlAllocation_terminate.c
 *
 * Code generation for function 'ControlAllocation_terminate'
 *
 */

/* Include files */
#include "ControlAllocation_terminate.h"
#include "ControlAllocation_data.h"
#include "_coder_ControlAllocation_mex.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void ControlAllocation_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void ControlAllocation_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (ControlAllocation_terminate.c) */
