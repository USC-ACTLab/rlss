#include "svmcvxwrapper_16pt_4obs.h"
#include "cvxgen/16pt4obspt/cvxgen/solver.h"
#include <iostream>

namespace solver_164 {

solver_164::Vars vars;
solver_164::Params params;
solver_164::Workspace work;
solver_164::Settings settings;

}

hyperplane _16pt4obspt_svm(const vectoreuc& pt1, const vectoreuc& pt2,
                          const vectoreuc& pt3, const vectoreuc& pt4,
                          const vectoreuc& pt5, const vectoreuc& pt6,
                          const vectoreuc& pt7, const vectoreuc& pt8,
                          const vectoreuc& pt9, const vectoreuc& pt10,
                          const vectoreuc& pt11, const vectoreuc& pt12,
                          const vectoreuc& pt13, const vectoreuc& pt14,
                          const vectoreuc& pt15, const vectoreuc& pt16,
                          const vectoreuc& obs1, const vectoreuc& obs2,
                          const vectoreuc& obs3, const vectoreuc& obs4)
{
  solver_164::set_defaults();
  solver_164::settings.eps = 1e-8;
  solver_164::settings.max_iters = 100;
  solver_164::settings.resid_tol = 1e-8;
  solver_164::settings.kkt_reg = 1e-4;
  solver_164::setup_indexing();

  solver_164::params.H[0] = 1;
  solver_164::params.H[1] = 1;
  solver_164::params.H[2] = 0;

  solver_164::params.A[0] = pt1.crds[0];
  solver_164::params.A[1] = pt2.crds[0];
  solver_164::params.A[2] = pt3.crds[0];
  solver_164::params.A[3] = pt4.crds[0];
  solver_164::params.A[4] = pt5.crds[0];
  solver_164::params.A[5] = pt6.crds[0];
  solver_164::params.A[6] = pt7.crds[0];
  solver_164::params.A[7] = pt8.crds[0];
  solver_164::params.A[8] = pt9.crds[0];
  solver_164::params.A[9] = pt10.crds[0];
  solver_164::params.A[10] = pt11.crds[0];
  solver_164::params.A[11] = pt12.crds[0];
  solver_164::params.A[12] = pt13.crds[0];
  solver_164::params.A[13] = pt14.crds[0];
  solver_164::params.A[14] = pt15.crds[0];
  solver_164::params.A[15] = pt16.crds[0];
  solver_164::params.A[16] = -obs1.crds[0];
  solver_164::params.A[17] = -obs2.crds[0];
  solver_164::params.A[18] = -obs3.crds[0];
  solver_164::params.A[19] = -obs4.crds[0];


  solver_164::params.A[20] = pt1.crds[1];
  solver_164::params.A[21] = pt2.crds[1];
  solver_164::params.A[22] = pt3.crds[1];
  solver_164::params.A[23] = pt4.crds[1];
  solver_164::params.A[24] = pt5.crds[1];
  solver_164::params.A[25] = pt6.crds[1];
  solver_164::params.A[26] = pt7.crds[1];
  solver_164::params.A[27] = pt8.crds[1];
  solver_164::params.A[28] = pt9.crds[1];
  solver_164::params.A[29] = pt10.crds[1];
  solver_164::params.A[30] = pt11.crds[1];
  solver_164::params.A[31] = pt12.crds[1];
  solver_164::params.A[32] = pt13.crds[1];
  solver_164::params.A[33] = pt14.crds[1];
  solver_164::params.A[34] = pt15.crds[1];
  solver_164::params.A[35] = pt16.crds[1];
  solver_164::params.A[36] = -obs1.crds[1];
  solver_164::params.A[37] = -obs2.crds[1];
  solver_164::params.A[38] = -obs3.crds[1];
  solver_164::params.A[39] = -obs4.crds[1];

  solver_164::params.A[40] = -1;
  solver_164::params.A[41] = -1;
  solver_164::params.A[42] = -1;
  solver_164::params.A[43] = -1;
  solver_164::params.A[44] = -1;
  solver_164::params.A[45] = -1;
  solver_164::params.A[46] = -1;
  solver_164::params.A[47] = -1;
  solver_164::params.A[48] = -1;
  solver_164::params.A[49] = -1;
  solver_164::params.A[50] = -1;
  solver_164::params.A[51] = -1;
  solver_164::params.A[52] = -1;
  solver_164::params.A[53] = -1;
  solver_164::params.A[54] = -1;
  solver_164::params.A[55] = -1;
  solver_164::params.A[56] = 1;
  solver_164::params.A[57] = 1;
  solver_164::params.A[58] = 1;
  solver_164::params.A[59] = 1;

  solver_164::settings.verbose = 0;

  int num_iters = solver_164::solve();

  // std::cout << "8-4 converged: " << solver_84::work.converged << std::endl;
  if (!solver_164::work.converged) {
      //std::cerr << "32-4 not converged" << std::endl;
  }

  double nx = solver_164::vars.w[0];
  double ny = solver_164::vars.w[1];
  double d = solver_164::vars.w[2];// + (2*solver_164::work.converged-1);// + solver_164::work.converged;

  vectoreuc normal(2);
  normal[0] = nx;
  normal[1] = ny;
  double length = normal.L2norm();

  hyperplane hp;
  hp.distance = d/length;
  hp.normal = normal.normalized();

  return hp;
}
