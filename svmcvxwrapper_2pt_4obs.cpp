#include "svmcvxwrapper_2pt_4obs.h"
#include "cvxgen/2pt4obspt/cvxgen/solver.h"

namespace solver_24 {
solver_24::Vars vars;
solver_24::Params params;
solver_24::Workspace work;
solver_24::Settings settings;

}

hyperplane _2pt4obspt_svm(const vectoreuc& pt1, const vectoreuc& pt2,
                          const vectoreuc& obs1, const vectoreuc& obs2,
                          const vectoreuc& obs3, const vectoreuc& obs4)
{

    solver_24::set_defaults();
    solver_24::setup_indexing();

    solver_24::params.H[0] = 1;
    solver_24::params.H[1] = 1;
    solver_24::params.H[2] = 0;

    solver_24::params.A[0] = pt1.crds[0];
    solver_24::params.A[1] = pt2.crds[0];
    solver_24::params.A[2] = -obs1.crds[0];
    solver_24::params.A[3] = -obs2.crds[0];
    solver_24::params.A[4] = -obs3.crds[0];
    solver_24::params.A[5] = -obs4.crds[0];
    solver_24::params.A[6] = pt1.crds[1];
    solver_24::params.A[7] = pt2.crds[1];
    solver_24::params.A[8] = -obs1.crds[1];
    solver_24::params.A[9] = -obs2.crds[1];
    solver_24::params.A[10] = -obs3.crds[1];
    solver_24::params.A[11] = -obs4.crds[1];
    solver_24::params.A[12] = -1;
    solver_24::params.A[13] = -1;
    solver_24::params.A[14] = 1;
    solver_24::params.A[15] = 1;
    solver_24::params.A[16] = 1;
    solver_24::params.A[17] = 1;

    solver_24::settings.verbose = 0;

    int num_iters = solver_24::solve();

    double nx = solver_24::vars.w[0];
    double ny = solver_24::vars.w[1];
    double d = solver_24::vars.w[2] + 1;

    vectoreuc normal(2);
    normal[0] = nx;
    normal[1] = ny;
    double length = normal.L2norm();

    hyperplane hp;
    hp.distance = d/length;
    hp.normal = normal.normalized();

    return hp;
}
