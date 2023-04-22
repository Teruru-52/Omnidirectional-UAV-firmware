/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_ControlAllocation_mex.c
 *
 * Code generation for function '_coder_ControlAllocation_mex'
 *
 */

/* Include files */
#include "_coder_ControlAllocation_mex.h"
#include "ControlAllocation_data.h"
#include "ControlAllocation_initialize.h"
#include "ControlAllocation_terminate.h"
#include "_coder_ControlAllocation_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void ControlAllocation_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T nrhs,
                                   const mxArray *prhs[3])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        17, "ControlAllocation");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 17,
                        "ControlAllocation");
  }
  /* Call the function. */
  ControlAllocation_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&ControlAllocation_atexit);
  /* Module initialization. */
  ControlAllocation_initialize();
  /* Dispatch the entry-point. */
  ControlAllocation_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  ControlAllocation_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "UTF-8", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_ControlAllocation_mex.c) */
