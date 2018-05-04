/* Produced by CVXGEN, 2018-05-04 18:05:55 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void solver_84::multbymA(double *lhs, double *rhs) {
}
void solver_84::multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
}
void solver_84::multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[12])-rhs[2]*(params.A[24]);
  lhs[1] = -rhs[0]*(params.A[1])-rhs[1]*(params.A[13])-rhs[2]*(params.A[25]);
  lhs[2] = -rhs[0]*(params.A[2])-rhs[1]*(params.A[14])-rhs[2]*(params.A[26]);
  lhs[3] = -rhs[0]*(params.A[3])-rhs[1]*(params.A[15])-rhs[2]*(params.A[27]);
  lhs[4] = -rhs[0]*(params.A[4])-rhs[1]*(params.A[16])-rhs[2]*(params.A[28]);
  lhs[5] = -rhs[0]*(params.A[5])-rhs[1]*(params.A[17])-rhs[2]*(params.A[29]);
  lhs[6] = -rhs[0]*(params.A[6])-rhs[1]*(params.A[18])-rhs[2]*(params.A[30]);
  lhs[7] = -rhs[0]*(params.A[7])-rhs[1]*(params.A[19])-rhs[2]*(params.A[31]);
  lhs[8] = -rhs[0]*(params.A[8])-rhs[1]*(params.A[20])-rhs[2]*(params.A[32]);
  lhs[9] = -rhs[0]*(params.A[9])-rhs[1]*(params.A[21])-rhs[2]*(params.A[33]);
  lhs[10] = -rhs[0]*(params.A[10])-rhs[1]*(params.A[22])-rhs[2]*(params.A[34]);
  lhs[11] = -rhs[0]*(params.A[11])-rhs[1]*(params.A[23])-rhs[2]*(params.A[35]);
}
void solver_84::multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[1])-rhs[2]*(params.A[2])-rhs[3]*(params.A[3])-rhs[4]*(params.A[4])-rhs[5]*(params.A[5])-rhs[6]*(params.A[6])-rhs[7]*(params.A[7])-rhs[8]*(params.A[8])-rhs[9]*(params.A[9])-rhs[10]*(params.A[10])-rhs[11]*(params.A[11]);
  lhs[1] = -rhs[0]*(params.A[12])-rhs[1]*(params.A[13])-rhs[2]*(params.A[14])-rhs[3]*(params.A[15])-rhs[4]*(params.A[16])-rhs[5]*(params.A[17])-rhs[6]*(params.A[18])-rhs[7]*(params.A[19])-rhs[8]*(params.A[20])-rhs[9]*(params.A[21])-rhs[10]*(params.A[22])-rhs[11]*(params.A[23]);
  lhs[2] = -rhs[0]*(params.A[24])-rhs[1]*(params.A[25])-rhs[2]*(params.A[26])-rhs[3]*(params.A[27])-rhs[4]*(params.A[28])-rhs[5]*(params.A[29])-rhs[6]*(params.A[30])-rhs[7]*(params.A[31])-rhs[8]*(params.A[32])-rhs[9]*(params.A[33])-rhs[10]*(params.A[34])-rhs[11]*(params.A[35]);
}
void solver_84::multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.H[0]);
  lhs[1] = rhs[1]*(2*params.H[1]);
  lhs[2] = rhs[2]*(2*params.H[2]);
}
void solver_84::fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
}
void solver_84::fillh(void) {
  work.h[0] = -1;
  work.h[1] = -1;
  work.h[2] = -1;
  work.h[3] = -1;
  work.h[4] = -1;
  work.h[5] = -1;
  work.h[6] = -1;
  work.h[7] = -1;
  work.h[8] = -1;
  work.h[9] = -1;
  work.h[10] = -1;
  work.h[11] = -1;
}
void solver_84::fillb(void) {
}
void solver_84::pre_ops(void) {
}
