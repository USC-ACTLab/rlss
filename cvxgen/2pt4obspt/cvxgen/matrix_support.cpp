/* Produced by CVXGEN, 2018-05-04 17:52:01 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void solver_24::multbymA(double *lhs, double *rhs) {
}
void solver_24::multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
}
void solver_24::multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[6])-rhs[2]*(params.A[12]);
  lhs[1] = -rhs[0]*(params.A[1])-rhs[1]*(params.A[7])-rhs[2]*(params.A[13]);
  lhs[2] = -rhs[0]*(params.A[2])-rhs[1]*(params.A[8])-rhs[2]*(params.A[14]);
  lhs[3] = -rhs[0]*(params.A[3])-rhs[1]*(params.A[9])-rhs[2]*(params.A[15]);
  lhs[4] = -rhs[0]*(params.A[4])-rhs[1]*(params.A[10])-rhs[2]*(params.A[16]);
  lhs[5] = -rhs[0]*(params.A[5])-rhs[1]*(params.A[11])-rhs[2]*(params.A[17]);
}
void solver_24::multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[1])-rhs[2]*(params.A[2])-rhs[3]*(params.A[3])-rhs[4]*(params.A[4])-rhs[5]*(params.A[5]);
  lhs[1] = -rhs[0]*(params.A[6])-rhs[1]*(params.A[7])-rhs[2]*(params.A[8])-rhs[3]*(params.A[9])-rhs[4]*(params.A[10])-rhs[5]*(params.A[11]);
  lhs[2] = -rhs[0]*(params.A[12])-rhs[1]*(params.A[13])-rhs[2]*(params.A[14])-rhs[3]*(params.A[15])-rhs[4]*(params.A[16])-rhs[5]*(params.A[17]);
}
void solver_24::multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.H[0]);
  lhs[1] = rhs[1]*(2*params.H[1]);
  lhs[2] = rhs[2]*(2*params.H[2]);
}
void solver_24::fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
}
void solver_24::fillh(void) {
  work.h[0] = -1;
  work.h[1] = -1;
  work.h[2] = -1;
  work.h[3] = -1;
  work.h[4] = -1;
  work.h[5] = -1;
}
void solver_24::fillb(void) {
}
void solver_24::pre_ops(void) {
}
