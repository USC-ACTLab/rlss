#include "svmcvxwrapper_32pt_4obs.h"
#include "cvxgen/32pt4obspt/cvxgen/solver.h"
#include <iostream>

namespace solver_324 {

solver_324::Vars vars;
solver_324::Params params;
solver_324::Workspace work;
solver_324::Settings settings;

}

hyperplane _32pt4obspt_svm(const vectoreuc& pt1, const vectoreuc& pt2,
                          const vectoreuc& pt3, const vectoreuc& pt4,
                          const vectoreuc& pt5, const vectoreuc& pt6,
                          const vectoreuc& pt7, const vectoreuc& pt8,
                          const vectoreuc& pt9, const vectoreuc& pt10,
                          const vectoreuc& pt11, const vectoreuc& pt12,
                          const vectoreuc& pt13, const vectoreuc& pt14,
                          const vectoreuc& pt15, const vectoreuc& pt16,
                          const vectoreuc& pt17, const vectoreuc& pt18,
                          const vectoreuc& pt19, const vectoreuc& pt20,
                          const vectoreuc& pt21, const vectoreuc& pt22,
                          const vectoreuc& pt23, const vectoreuc& pt24,
                          const vectoreuc& pt25, const vectoreuc& pt26,
                          const vectoreuc& pt27, const vectoreuc& pt28,
                          const vectoreuc& pt29, const vectoreuc& pt30,
                          const vectoreuc& pt31, const vectoreuc& pt32,
                          const vectoreuc& obs1, const vectoreuc& obs2,
                          const vectoreuc& obs3, const vectoreuc& obs4)
{
  solver_324::set_defaults();
  solver_324::setup_indexing();

  solver_324::params.H[0] = 1;
  solver_324::params.H[1] = 1;
  solver_324::params.H[2] = 0;

  solver_324::params.A[0] = pt1.crds[0];
  solver_324::params.A[1] = pt2.crds[0];
  solver_324::params.A[2] = pt3.crds[0];
  solver_324::params.A[3] = pt4.crds[0];
  solver_324::params.A[4] = pt5.crds[0];
  solver_324::params.A[5] = pt6.crds[0];
  solver_324::params.A[6] = pt7.crds[0];
  solver_324::params.A[7] = pt8.crds[0];
  solver_324::params.A[8] = pt9.crds[0];
  solver_324::params.A[9] = pt10.crds[0];
  solver_324::params.A[10] = pt11.crds[0];
  solver_324::params.A[11] = pt12.crds[0];
  solver_324::params.A[12] = pt13.crds[0];
  solver_324::params.A[13] = pt14.crds[0];
  solver_324::params.A[14] = pt15.crds[0];
  solver_324::params.A[15] = pt16.crds[0];
  solver_324::params.A[16] = pt17.crds[0];
  solver_324::params.A[17] = pt18.crds[0];
  solver_324::params.A[18] = pt19.crds[0];
  solver_324::params.A[19] = pt20.crds[0];
  solver_324::params.A[20] = pt21.crds[0];
  solver_324::params.A[21] = pt22.crds[0];
  solver_324::params.A[22] = pt23.crds[0];
  solver_324::params.A[23] = pt24.crds[0];
  solver_324::params.A[24] = pt25.crds[0];
  solver_324::params.A[25] = pt26.crds[0];
  solver_324::params.A[26] = pt27.crds[0];
  solver_324::params.A[27] = pt28.crds[0];
  solver_324::params.A[28] = pt29.crds[0];
  solver_324::params.A[29] = pt30.crds[0];
  solver_324::params.A[30] = pt31.crds[0];
  solver_324::params.A[31] = pt32.crds[0];
  solver_324::params.A[32] = -obs1.crds[0];
  solver_324::params.A[33] = -obs2.crds[0];
  solver_324::params.A[34] = -obs3.crds[0];
  solver_324::params.A[35] = -obs4.crds[0];


  solver_324::params.A[36] = pt1.crds[1];
  solver_324::params.A[37] = pt2.crds[1];
  solver_324::params.A[38] = pt3.crds[1];
  solver_324::params.A[39] = pt4.crds[1];
  solver_324::params.A[40] = pt5.crds[1];
  solver_324::params.A[41] = pt6.crds[1];
  solver_324::params.A[42] = pt7.crds[1];
  solver_324::params.A[43] = pt8.crds[1];
  solver_324::params.A[44] = pt9.crds[1];
  solver_324::params.A[45] = pt10.crds[1];
  solver_324::params.A[46] = pt11.crds[1];
  solver_324::params.A[47] = pt12.crds[1];
  solver_324::params.A[48] = pt13.crds[1];
  solver_324::params.A[49] = pt14.crds[1];
  solver_324::params.A[50] = pt15.crds[1];
  solver_324::params.A[51] = pt16.crds[1];
  solver_324::params.A[52] = pt17.crds[1];
  solver_324::params.A[53] = pt18.crds[1];
  solver_324::params.A[54] = pt19.crds[1];
  solver_324::params.A[55] = pt20.crds[1];
  solver_324::params.A[56] = pt21.crds[1];
  solver_324::params.A[57] = pt22.crds[1];
  solver_324::params.A[58] = pt23.crds[1];
  solver_324::params.A[59] = pt24.crds[1];
  solver_324::params.A[60] = pt25.crds[1];
  solver_324::params.A[61] = pt26.crds[1];
  solver_324::params.A[62] = pt27.crds[1];
  solver_324::params.A[63] = pt28.crds[1];
  solver_324::params.A[64] = pt29.crds[1];
  solver_324::params.A[65] = pt30.crds[1];
  solver_324::params.A[66] = pt31.crds[1];
  solver_324::params.A[67] = pt32.crds[1];
  solver_324::params.A[68] = -obs1.crds[1];
  solver_324::params.A[69] = -obs2.crds[1];
  solver_324::params.A[70] = -obs3.crds[1];
  solver_324::params.A[71] = -obs4.crds[1];

  solver_324::params.A[72] = -1;
  solver_324::params.A[73] = -1;
  solver_324::params.A[74] = -1;
  solver_324::params.A[75] = -1;
  solver_324::params.A[76] = -1;
  solver_324::params.A[77] = -1;
  solver_324::params.A[78] = -1;
  solver_324::params.A[79] = -1;
  solver_324::params.A[80] = -1;
  solver_324::params.A[81] = -1;
  solver_324::params.A[82] = -1;
  solver_324::params.A[83] = -1;
  solver_324::params.A[84] = -1;
  solver_324::params.A[85] = -1;
  solver_324::params.A[86] = -1;
  solver_324::params.A[87] = -1;
  solver_324::params.A[88] = -1;
  solver_324::params.A[89] = -1;
  solver_324::params.A[90] = -1;
  solver_324::params.A[91] = -1;
  solver_324::params.A[92] = -1;
  solver_324::params.A[93] = -1;
  solver_324::params.A[94] = -1;
  solver_324::params.A[95] = -1;
  solver_324::params.A[96] = -1;
  solver_324::params.A[97] = -1;
  solver_324::params.A[98] = -1;
  solver_324::params.A[99] = -1;
  solver_324::params.A[100] = -1;
  solver_324::params.A[101] = -1;
  solver_324::params.A[102] = -1;
  solver_324::params.A[103] = -1;
  solver_324::params.A[104] = 1;
  solver_324::params.A[105] = 1;
  solver_324::params.A[106] = 1;
  solver_324::params.A[107] = 1;

  solver_324::settings.verbose = 0;

  int num_iters = solver_324::solve();

  // std::cout << "8-4 converged: " << solver_84::work.converged << std::endl;
  if (!solver_324::work.converged) {
      std::cerr << "32-4 not converged" << std::endl;
  }

  double nx = solver_324::vars.w[0];
  double ny = solver_324::vars.w[1];
  double d = solver_324::vars.w[2] + solver_324::work.converged;

  vectoreuc normal(2);
  normal[0] = nx;
  normal[1] = ny;
  double length = normal.L2norm();

  hyperplane hp;
  hp.distance = d/length;
  hp.normal = normal.normalized();

  return hp;
}
