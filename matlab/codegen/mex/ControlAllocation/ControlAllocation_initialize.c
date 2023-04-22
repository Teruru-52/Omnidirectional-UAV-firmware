/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ControlAllocation_initialize.c
 *
 * Code generation for function 'ControlAllocation_initialize'
 *
 */

/* Include files */
#include "ControlAllocation_initialize.h"
#include "ControlAllocation_data.h"
#include "_coder_ControlAllocation_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void ControlAllocation_once(void);

/* Function Definitions */
static void ControlAllocation_once(void)
{
  mex_InitInfAndNan();
}

void ControlAllocation_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    ControlAllocation_once();
  }
}

/* End of code generation (ControlAllocation_initialize.c) */
