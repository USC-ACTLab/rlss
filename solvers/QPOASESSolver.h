#include "qpOASES.hpp"
#include "QPSolver.h"
#include "spline.h"

template<class T>

class QPOASESSolver : public QPSolver<T> {
   private:
   using QPMatrices = typename Spline<T, 3U>::QPMatrices;
   using Vector = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;
   bool set_max_time;
   qpOASES::Options options;
   qpOASES::real_t cputime;
   qpOASES::int_t nWSR;

   public:
   QPOASESSolver(bool set_max_time, double dt) {
    this->set_max_time = set_max_time;
    this->options.setToMPC();
    this->options.printLevel = qpOASES::PL_NONE;
    this->nWSR = 10000;
    this->cputime = dt;
   }

   void solve(const QPMatrices& combinedQP, bool& success, Vector& result) override {
     
     qpOASES::QProblem problem(combinedQP.x.rows(), combinedQP.A.rows());
      problem.setOptions(this->options);
        qpOASES::returnValue ret =
          problem.init(
            combinedQP.H.data(),
            combinedQP.g.data(),
            combinedQP.A.data(),
            combinedQP.lbX.data(),
            combinedQP.ubX.data(),
            combinedQP.lbA.data(),
            combinedQP.ubA.data(),
            this->nWSR,
            this->set_max_time ? &this->cputime: NULL,
            combinedQP.x.data()
          );
        qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(ret);
        success = simpleStatus == 0;
        qpOASES::int_t res_written = problem.getPrimalSolution(result.data());

   };
};