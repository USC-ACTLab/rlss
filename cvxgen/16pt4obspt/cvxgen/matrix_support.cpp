/* Produced by CVXGEN, 2018-07-02 18:28:37 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void solver_164::multbymA(double *lhs, double *rhs) {
}
void solver_164::multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
}
void solver_164::multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[20])-rhs[2]*(params.A[40]);
  lhs[1] = -rhs[0]*(params.A[1])-rhs[1]*(params.A[21])-rhs[2]*(params.A[41]);
  lhs[2] = -rhs[0]*(params.A[2])-rhs[1]*(params.A[22])-rhs[2]*(params.A[42]);
  lhs[3] = -rhs[0]*(params.A[3])-rhs[1]*(params.A[23])-rhs[2]*(params.A[43]);
  lhs[4] = -rhs[0]*(params.A[4])-rhs[1]*(params.A[24])-rhs[2]*(params.A[44]);
  lhs[5] = -rhs[0]*(params.A[5])-rhs[1]*(params.A[25])-rhs[2]*(params.A[45]);
  lhs[6] = -rhs[0]*(params.A[6])-rhs[1]*(params.A[26])-rhs[2]*(params.A[46]);
  lhs[7] = -rhs[0]*(params.A[7])-rhs[1]*(params.A[27])-rhs[2]*(params.A[47]);
  lhs[8] = -rhs[0]*(params.A[8])-rhs[1]*(params.A[28])-rhs[2]*(params.A[48]);
  lhs[9] = -rhs[0]*(params.A[9])-rhs[1]*(params.A[29])-rhs[2]*(params.A[49]);
  lhs[10] = -rhs[0]*(params.A[10])-rhs[1]*(params.A[30])-rhs[2]*(params.A[50]);
  lhs[11] = -rhs[0]*(params.A[11])-rhs[1]*(params.A[31])-rhs[2]*(params.A[51]);
  lhs[12] = -rhs[0]*(params.A[12])-rhs[1]*(params.A[32])-rhs[2]*(params.A[52]);
  lhs[13] = -rhs[0]*(params.A[13])-rhs[1]*(params.A[33])-rhs[2]*(params.A[53]);
  lhs[14] = -rhs[0]*(params.A[14])-rhs[1]*(params.A[34])-rhs[2]*(params.A[54]);
  lhs[15] = -rhs[0]*(params.A[15])-rhs[1]*(params.A[35])-rhs[2]*(params.A[55]);
  lhs[16] = -rhs[0]*(params.A[16])-rhs[1]*(params.A[36])-rhs[2]*(params.A[56]);
  lhs[17] = -rhs[0]*(params.A[17])-rhs[1]*(params.A[37])-rhs[2]*(params.A[57]);
  lhs[18] = -rhs[0]*(params.A[18])-rhs[1]*(params.A[38])-rhs[2]*(params.A[58]);
  lhs[19] = -rhs[0]*(params.A[19])-rhs[1]*(params.A[39])-rhs[2]*(params.A[59]);
}
void solver_164::multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.A[0])-rhs[1]*(params.A[1])-rhs[2]*(params.A[2])-rhs[3]*(params.A[3])-rhs[4]*(params.A[4])-rhs[5]*(params.A[5])-rhs[6]*(params.A[6])-rhs[7]*(params.A[7])-rhs[8]*(params.A[8])-rhs[9]*(params.A[9])-rhs[10]*(params.A[10])-rhs[11]*(params.A[11])-rhs[12]*(params.A[12])-rhs[13]*(params.A[13])-rhs[14]*(params.A[14])-rhs[15]*(params.A[15])-rhs[16]*(params.A[16])-rhs[17]*(params.A[17])-rhs[18]*(params.A[18])-rhs[19]*(params.A[19]);
  lhs[1] = -rhs[0]*(params.A[20])-rhs[1]*(params.A[21])-rhs[2]*(params.A[22])-rhs[3]*(params.A[23])-rhs[4]*(params.A[24])-rhs[5]*(params.A[25])-rhs[6]*(params.A[26])-rhs[7]*(params.A[27])-rhs[8]*(params.A[28])-rhs[9]*(params.A[29])-rhs[10]*(params.A[30])-rhs[11]*(params.A[31])-rhs[12]*(params.A[32])-rhs[13]*(params.A[33])-rhs[14]*(params.A[34])-rhs[15]*(params.A[35])-rhs[16]*(params.A[36])-rhs[17]*(params.A[37])-rhs[18]*(params.A[38])-rhs[19]*(params.A[39]);
  lhs[2] = -rhs[0]*(params.A[40])-rhs[1]*(params.A[41])-rhs[2]*(params.A[42])-rhs[3]*(params.A[43])-rhs[4]*(params.A[44])-rhs[5]*(params.A[45])-rhs[6]*(params.A[46])-rhs[7]*(params.A[47])-rhs[8]*(params.A[48])-rhs[9]*(params.A[49])-rhs[10]*(params.A[50])-rhs[11]*(params.A[51])-rhs[12]*(params.A[52])-rhs[13]*(params.A[53])-rhs[14]*(params.A[54])-rhs[15]*(params.A[55])-rhs[16]*(params.A[56])-rhs[17]*(params.A[57])-rhs[18]*(params.A[58])-rhs[19]*(params.A[59]);
}
void solver_164::multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.H[0]);
  lhs[1] = rhs[1]*(2*params.H[1]);
  lhs[2] = rhs[2]*(2*params.H[2]);
}
void solver_164::fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
}
void solver_164::fillh(void) {
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
  work.h[12] = -1;
  work.h[13] = -1;
  work.h[14] = -1;
  work.h[15] = -1;
  work.h[16] = -1;
  work.h[17] = -1;
  work.h[18] = -1;
  work.h[19] = -1;
}
void solver_164::fillb(void) {
}
void solver_164::pre_ops(void) {
}
