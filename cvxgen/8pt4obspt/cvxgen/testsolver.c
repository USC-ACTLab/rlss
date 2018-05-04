/* Produced by CVXGEN, 2018-05-04 18:05:55 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.H[0] = 1.5507979025745755;
  params.H[1] = 1.7081478226181048;
  params.H[2] = 1.2909047389129444;
  params.A[0] = 0.04331042079065206;
  params.A[1] = 1.5717878173906188;
  params.A[2] = 1.5851723557337523;
  params.A[3] = -1.497658758144655;
  params.A[4] = -1.171028487447253;
  params.A[5] = -1.7941311867966805;
  params.A[6] = -0.23676062539745413;
  params.A[7] = -1.8804951564857322;
  params.A[8] = -0.17266710242115568;
  params.A[9] = 0.596576190459043;
  params.A[10] = -0.8860508694080989;
  params.A[11] = 0.7050196079205251;
  params.A[12] = 0.3634512696654033;
  params.A[13] = -1.9040724704913385;
  params.A[14] = 0.23541635196352795;
  params.A[15] = -0.9629902123701384;
  params.A[16] = -0.3395952119597214;
  params.A[17] = -0.865899672914725;
  params.A[18] = 0.7725516732519853;
  params.A[19] = -0.23818512931704205;
  params.A[20] = -1.372529046100147;
  params.A[21] = 0.17859607212737894;
  params.A[22] = 1.1212590580454682;
  params.A[23] = -0.774545870495281;
  params.A[24] = -1.1121684642712744;
  params.A[25] = -0.44811496977740495;
  params.A[26] = 1.7455345994417217;
  params.A[27] = 1.9039816898917352;
  params.A[28] = 0.6895347036512547;
  params.A[29] = 1.6113364341535923;
  params.A[30] = 1.383003485172717;
  params.A[31] = -0.48802383468444344;
  params.A[32] = -1.631131964513103;
  params.A[33] = 0.6136436100941447;
  params.A[34] = 0.2313630495538037;
  params.A[35] = -0.5537409477496875;
}
