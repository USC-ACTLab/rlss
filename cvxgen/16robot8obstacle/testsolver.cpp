/* Produced by CVXGEN, 2019-01-04 19:48:16 -0500.  */
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
  params.H[3] = 1.510827605197663;
  params.A[0] = 1.5717878173906188;
  params.A[1] = 1.5851723557337523;
  params.A[2] = -1.497658758144655;
  params.A[3] = -1.171028487447253;
  params.A[4] = -1.7941311867966805;
  params.A[5] = -0.23676062539745413;
  params.A[6] = -1.8804951564857322;
  params.A[7] = -0.17266710242115568;
  params.A[8] = 0.596576190459043;
  params.A[9] = -0.8860508694080989;
  params.A[10] = 0.7050196079205251;
  params.A[11] = 0.3634512696654033;
  params.A[12] = -1.9040724704913385;
  params.A[13] = 0.23541635196352795;
  params.A[14] = -0.9629902123701384;
  params.A[15] = -0.3395952119597214;
  params.A[16] = -0.865899672914725;
  params.A[17] = 0.7725516732519853;
  params.A[18] = -0.23818512931704205;
  params.A[19] = -1.372529046100147;
  params.A[20] = 0.17859607212737894;
  params.A[21] = 1.1212590580454682;
  params.A[22] = -0.774545870495281;
  params.A[23] = -1.1121684642712744;
  params.A[24] = -0.44811496977740495;
  params.A[25] = 1.7455345994417217;
  params.A[26] = 1.9039816898917352;
  params.A[27] = 0.6895347036512547;
  params.A[28] = 1.6113364341535923;
  params.A[29] = 1.383003485172717;
  params.A[30] = -0.48802383468444344;
  params.A[31] = -1.631131964513103;
  params.A[32] = 0.6136436100941447;
  params.A[33] = 0.2313630495538037;
  params.A[34] = -0.5537409477496875;
  params.A[35] = -1.0997819806406723;
  params.A[36] = -0.3739203344950055;
  params.A[37] = -0.12423900520332376;
  params.A[38] = -0.923057686995755;
  params.A[39] = -0.8328289030982696;
  params.A[40] = -0.16925440270808823;
  params.A[41] = 1.442135651787706;
  params.A[42] = 0.34501161787128565;
  params.A[43] = -0.8660485502711608;
  params.A[44] = -0.8880899735055947;
  params.A[45] = -0.1815116979122129;
  params.A[46] = -1.17835862158005;
  params.A[47] = -1.1944851558277074;
  params.A[48] = 0.05614023926976763;
  params.A[49] = -1.6510825248767813;
  params.A[50] = -0.06565787059365391;
  params.A[51] = -0.5512951504486665;
  params.A[52] = 0.8307464872626844;
  params.A[53] = 0.9869848924080182;
  params.A[54] = 0.7643716874230573;
  params.A[55] = 0.7567216550196565;
  params.A[56] = -0.5055995034042868;
  params.A[57] = 0.6725392189410702;
  params.A[58] = -0.6406053441727284;
  params.A[59] = 0.29117547947550015;
  params.A[60] = -0.6967713677405021;
  params.A[61] = -0.21941980294587182;
  params.A[62] = -1.753884276680243;
  params.A[63] = -1.0292983112626475;
  params.A[64] = 1.8864104246942706;
  params.A[65] = -1.077663182579704;
  params.A[66] = 0.7659100437893209;
  params.A[67] = 0.6019074328549583;
  params.A[68] = 0.8957565577499285;
  params.A[69] = -0.09964555746227477;
  params.A[70] = 0.38665509840745127;
  params.A[71] = -1.7321223042686946;
  params.A[72] = -1.7097514487110663;
  params.A[73] = -1.2040958948116867;
  params.A[74] = -1.3925560119658358;
  params.A[75] = -1.5995826216742213;
  params.A[76] = -1.4828245415645833;
  params.A[77] = 0.21311092723061398;
  params.A[78] = -1.248740700304487;
  params.A[79] = 1.808404972124833;
  params.A[80] = 0.7264471152297065;
  params.A[81] = 0.16407869343908477;
  params.A[82] = 0.8287224032315907;
  params.A[83] = -0.9444533161899464;
  params.A[84] = 1.7069027370149112;
  params.A[85] = 1.3567722311998827;
  params.A[86] = 0.9052779937121489;
  params.A[87] = -0.07904017565835986;
  params.A[88] = 1.3684127435065871;
  params.A[89] = 0.979009293697437;
  params.A[90] = 0.6413036255984501;
  params.A[91] = 1.6559010680237511;
  params.A[92] = 0.5346622551502991;
  params.A[93] = -0.5362376605895625;
  params.A[94] = 0.2113782926017822;
  params.A[95] = -1.2144776931994525;
}
