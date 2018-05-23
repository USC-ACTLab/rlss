#include "svmcvxwrapper_8pt_4obs.h"
#include "cvxgen/8pt4obspt/cvxgen/solver.h"
#include <iostream>

namespace solver_84 {

solver_84::Vars vars;
solver_84::Params params;
solver_84::Workspace work;
solver_84::Settings settings;

}

hyperplane _8pt4obspt_svm(const vectoreuc& pt1, const vectoreuc& pt2,
                          const vectoreuc& pt3, const vectoreuc& pt4,
                          const vectoreuc& pt5, const vectoreuc& pt6,
                          const vectoreuc& pt7, const vectoreuc& pt8,
                          const vectoreuc& obs1, const vectoreuc& obs2,
                          const vectoreuc& obs3, const vectoreuc& obs4)
{
  solver_84::set_defaults();
  solver_84::settings.eps = 1e-8;
  solver_84::settings.max_iters = 100;
  solver_84::settings.resid_tol = 1e-8;
  solver_84::settings.kkt_reg = 1e-4;
  solver_84::setup_indexing();

  solver_84::params.H[0] = 1;
  solver_84::params.H[1] = 1;
  solver_84::params.H[2] = 0;

  solver_84::params.A[0] = pt1.crds[0];
  solver_84::params.A[1] = pt2.crds[0];
  solver_84::params.A[2] = pt3.crds[0];
  solver_84::params.A[3] = pt4.crds[0];
  solver_84::params.A[4] = pt5.crds[0];
  solver_84::params.A[5] = pt6.crds[0];
  solver_84::params.A[6] = pt7.crds[0];
  solver_84::params.A[7] = pt8.crds[0];
  solver_84::params.A[8] = -obs1.crds[0];
  solver_84::params.A[9] = -obs2.crds[0];
  solver_84::params.A[10] = -obs3.crds[0];
  solver_84::params.A[11] = -obs4.crds[0];

  solver_84::params.A[12] = pt1.crds[1];
  solver_84::params.A[13] = pt2.crds[1];
  solver_84::params.A[14] = pt3.crds[1];
  solver_84::params.A[15] = pt4.crds[1];
  solver_84::params.A[16] = pt5.crds[1];
  solver_84::params.A[17] = pt6.crds[1];
  solver_84::params.A[18] = pt7.crds[1];
  solver_84::params.A[19] = pt8.crds[1];
  solver_84::params.A[20] = -obs1.crds[1];
  solver_84::params.A[21] = -obs2.crds[1];
  solver_84::params.A[22] = -obs3.crds[1];
  solver_84::params.A[23] = -obs4.crds[1];

  solver_84::params.A[24] = -1;
  solver_84::params.A[25] = -1;
  solver_84::params.A[26] = -1;
  solver_84::params.A[27] = -1;
  solver_84::params.A[28] = -1;
  solver_84::params.A[29] = -1;
  solver_84::params.A[30] = -1;
  solver_84::params.A[31] = -1;
  solver_84::params.A[32] = 1;
  solver_84::params.A[33] = 1;
  solver_84::params.A[34] = 1;
  solver_84::params.A[35] = 1;

  solver_84::settings.verbose = 0;

  int num_iters = solver_84::solve();

  // std::cout << "8-4 converged: " << solver_84::work.converged << std::endl;
  if (!solver_84::work.converged) {
      //std::cerr << "8-4 not converged" << std::endl;
  }

  double nx = solver_84::vars.w[0];
  double ny = solver_84::vars.w[1];
  double d = solver_84::vars.w[2];//  + (2*solver_84::work.converged-0.01);// + solver_84::work.converged;

  vectoreuc normal(2);
  normal[0] = nx;
  normal[1] = ny;
  double length = normal.L2norm();

  hyperplane hp;
  hp.distance = d/length;
  hp.normal = normal.normalized();

  return hp;
}
