#include "qpOASES.hpp"
#include "QPSolver.h"
#include "spline.h"


namespace ACT {
template<class T>
class QPOASESSolver : public QPSolver<T> {

  public:
  struct QPOASESOptions {
    bool set_max_time;
    qpOASES::real_t cputime; 
    qpOASES::int_t nWSR;
    qpOASES::Options qpOptions;
    QPOASESOptions() {}
    QPOASESOptions(bool set_max_time, qpOASES::real_t cputime, qpOASES::int_t nWSR, qpOASES::Options qpOptions){
      this->set_max_time = set_max_time;
      this-> cputime = cputime;
      this-> nWSR = nWSR;
      this-> qpOptions = qpOptions;
    }

    qpOASES::Options& getQPoptions() {
      return qpOptions;
    }

  };

  private:
  QPOASESOptions mQPOASESOptions;
  using QPMatrices = typename Spline<T, 3U>::QPMatrices;
  using Vector = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

  public:
  QPOASESSolver(QPOASESOptions opt) {
    this->mQPOASESOptions = QPOASESOptions(opt.set_max_time,opt.cputime,opt.nWSR,opt.qpOptions);
   }

   QPOASESSolver (qpOASES::real_t cputime, bool set_max_time) {
     qpOASES::Options options;
     options.setToMPC();
     options.printLevel = qpOASES::PL_NONE;
     this->mQPOASESOptions = QPOASESOptions(set_max_time,cputime,1000,options);
   }

   bool solve(const QPMatrices& combinedQP, Vector& result) override {
     assert(result.rows()==combinedQP.x.rows());
     qpOASES::QProblem problem(combinedQP.x.rows(), combinedQP.A.rows());
      problem.setOptions(this->mQPOASESOptions.qpOptions);
        qpOASES::returnValue ret =
          problem.init(
            combinedQP.H.data(),
            combinedQP.g.data(),
            combinedQP.A.data(),
            combinedQP.lbX.data(),
            combinedQP.ubX.data(),
            combinedQP.lbA.data(),
            combinedQP.ubA.data(),
            this->mQPOASESOptions.nWSR,
            this->mQPOASESOptions.set_max_time ? &this->mQPOASESOptions.cputime: NULL,
            combinedQP.x.data()
          );
        qpOASES::int_t simpleStatus = qpOASES::getSimpleStatus(ret);
        bool success = simpleStatus == 0;
        problem.getPrimalSolution(result.data());
        return success;
   };

   bool getSetMaxTime(){
     return this->mQPOASESOptions.set_max_time;
   }

   void setSetMaxTime(bool smt){
     this->mQPOASESOptions.set_max_time = smt;
   }

   qpOASES::real_t getCputime(){
     return this->mQPOASESOptions.cputime;
   }

   void setCputime( qpOASES::real_t tm){
     this->mQPOASESOptions.cputime = tm;
   }

   qpOASES::int_t getNWSR(){
     return this->mQPOASESOptions.nWSR;
   }

   void setNWSR(qpOASES::int_t nwsr){
     this->mQPOASESOptions.nWSR = nwsr;
   }

   qpOASES::Options& getqpOASESOptions() {
     return (this->mQPOASESOptions.getQPoptions());
   }
};
};