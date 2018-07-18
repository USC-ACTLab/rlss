#include "svmcvxwrapper_20pt_4obs.h"
#include "cvxgen/20pt4obspt/cvxgen/solver.h"
#include <iostream>

namespace solver_204 {

solver_204::Vars vars;
solver_204::Params params;
solver_204::Workspace work;
solver_204::Settings settings;

}

hyperplane _20pt4obspt_svm(const vectoreuc& pt1, const vectoreuc& pt2,
                          const vectoreuc& pt3, const vectoreuc& pt4,
                          const vectoreuc& pt5, const vectoreuc& pt6,
                          const vectoreuc& pt7, const vectoreuc& pt8,
                          const vectoreuc& pt9, const vectoreuc& pt10,
                          const vectoreuc& pt11, const vectoreuc& pt12,
                          const vectoreuc& pt13, const vectoreuc& pt14,
                          const vectoreuc& pt15, const vectoreuc& pt16,
                          const vectoreuc& pt17, const vectoreuc& pt18,
                          const vectoreuc& pt19, const vectoreuc& pt20,
                          const vectoreuc& obs1, const vectoreuc& obs2,
                          const vectoreuc& obs3, const vectoreuc& obs4)
{
  solver_204::set_defaults();
  solver_204::settings.eps = 1e-8;
  solver_204::settings.max_iters = 100;
  solver_204::settings.resid_tol = 1e-8;
  solver_204::settings.kkt_reg = 1e-4;
  solver_204::setup_indexing();

  solver_204::params.H[0] = 1;
  solver_204::params.H[1] = 1;
  solver_204::params.H[2] = 0;

  solver_204::params.A[0] = pt1.crds[0];
  solver_204::params.A[1] = pt2.crds[0];
  solver_204::params.A[2] = pt3.crds[0];
  solver_204::params.A[3] = pt4.crds[0];
  solver_204::params.A[4] = pt5.crds[0];
  solver_204::params.A[5] = pt6.crds[0];
  solver_204::params.A[6] = pt7.crds[0];
  solver_204::params.A[7] = pt8.crds[0];
  solver_204::params.A[8] = pt9.crds[0];
  solver_204::params.A[9] = pt10.crds[0];
  solver_204::params.A[10] = pt11.crds[0];
  solver_204::params.A[11] = pt12.crds[0];
  solver_204::params.A[12] = pt13.crds[0];
  solver_204::params.A[13] = pt14.crds[0];
  solver_204::params.A[14] = pt15.crds[0];
  solver_204::params.A[15] = pt16.crds[0];
  solver_204::params.A[16] = pt17.crds[0];
  solver_204::params.A[17] = pt18.crds[0];
  solver_204::params.A[18] = pt19.crds[0];
  solver_204::params.A[19] = pt20.crds[0];
  solver_204::params.A[20] = -obs1.crds[0];
  solver_204::params.A[21] = -obs2.crds[0];
  solver_204::params.A[22] = -obs3.crds[0];
  solver_204::params.A[23] = -obs4.crds[0];


  solver_204::params.A[24] = pt1.crds[1];
  solver_204::params.A[25] = pt2.crds[1];
  solver_204::params.A[26] = pt3.crds[1];
  solver_204::params.A[27] = pt4.crds[1];
  solver_204::params.A[28] = pt5.crds[1];
  solver_204::params.A[29] = pt6.crds[1];
  solver_204::params.A[30] = pt7.crds[1];
  solver_204::params.A[31] = pt8.crds[1];
  solver_204::params.A[32] = pt9.crds[1];
  solver_204::params.A[33] = pt10.crds[1];
  solver_204::params.A[34] = pt11.crds[1];
  solver_204::params.A[35] = pt12.crds[1];
  solver_204::params.A[36] = pt13.crds[1];
  solver_204::params.A[37] = pt14.crds[1];
  solver_204::params.A[38] = pt15.crds[1];
  solver_204::params.A[39] = pt16.crds[1];
  solver_204::params.A[40] = pt17.crds[1];
  solver_204::params.A[41] = pt18.crds[1];
  solver_204::params.A[42] = pt19.crds[1];
  solver_204::params.A[43] = pt20.crds[1];
  solver_204::params.A[44] = -obs1.crds[1];
  solver_204::params.A[45] = -obs2.crds[1];
  solver_204::params.A[46] = -obs3.crds[1];
  solver_204::params.A[47] = -obs4.crds[1];

  solver_204::params.A[48] = -1;
  solver_204::params.A[49] = -1;
  solver_204::params.A[50] = -1;
  solver_204::params.A[51] = -1;
  solver_204::params.A[52] = -1;
  solver_204::params.A[53] = -1;
  solver_204::params.A[54] = -1;
  solver_204::params.A[55] = -1;
  solver_204::params.A[56] = -1;
  solver_204::params.A[57] = -1;
  solver_204::params.A[58] = -1;
  solver_204::params.A[59] = -1;
  solver_204::params.A[60] = -1;
  solver_204::params.A[61] = -1;
  solver_204::params.A[62] = -1;
  solver_204::params.A[63] = -1;
  solver_204::params.A[64] = -1;
  solver_204::params.A[65] = -1;
  solver_204::params.A[66] = -1;
  solver_204::params.A[67] = -1;
  solver_204::params.A[68] = 1;
  solver_204::params.A[69] = 1;
  solver_204::params.A[70] = 1;
  solver_204::params.A[71] = 1;

  solver_204::settings.verbose = 0;

  int num_iters = solver_204::solve();

  // std::cout << "8-4 converged: " << solver_84::work.converged << std::endl;
  if (!solver_204::work.converged) {
      //std::cerr << "32-4 not converged" << std::endl;
  }

  double nx = solver_204::vars.w[0];
  double ny = solver_204::vars.w[1];
  double d = solver_204::vars.w[2];// + (2*solver_204::work.converged-1);// + solver_204::work.converged;

  vectoreuc normal(2);
  normal[0] = nx;
  normal[1] = ny;
  double length = normal.L2norm();

  hyperplane hp;
  hp.distance = d/length;
  hp.normal = normal.normalized();

  return hp;
}
