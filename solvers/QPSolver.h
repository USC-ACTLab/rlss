#include "spline.h"

template<class T>
class QPSolver {
   public:
   using Vector = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;
   using QPMatrices = typename Spline<T, 3U>::QPMatrices;
   virtual bool solve(const QPMatrices& qp, Vector& result) = 0;
};