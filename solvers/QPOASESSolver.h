#include "qpOASES.hpp"
#include "QPSolver.h"
#include "spline.h"

struct QPOASESOptions {
  bool set_max_time;
  qpOASES::real_t cputime; 
  qpOASES::int_t nWSR;
  qpOASES::Options qpOptions;

  QPOASESOptions(bool set_max_time, qpOASES::real_t cputime, qpOASES::int_t nWSR, qpOASES::Options qpOptions){
    this->set_max_time = set_max_time;
    this-> cputime = cputime;
    this-> nWSR = nWSR;
    this-> qpOptions = qpOptions;
  }
};

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
   QPOASESSolver(QPOASESOptions opt) {
    this->set_max_time = opt.set_max_time;
    this->options = opt.qpOptions;
    this->nWSR = opt.nWSR;
    this->cputime = opt.cputime;
   }

   bool solve(const QPMatrices& combinedQP, Vector& result) override {
     assert(result.rows()==combinedQP.x.rows());
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
        bool success = simpleStatus == 0;
        problem.getPrimalSolution(result.data());
        return success;
   };

   bool getSetMaxTime(){
     return this->set_max_time;
   }

   void setSetMaxTime(bool smt){
     this->set_max_time = smt;
   }

   qpOASES::real_t getCputime(){
     return this->cputime;
   }

   void setCputime( qpOASES::real_t tm){
     this->cputime = tm;
   }

   qpOASES::int_t getNWSR(){
     return this->nWSR;
   }

   void setNWSR(qpOASES::int_t nwsr){
     this->nWSR = nwsr;
   }




};