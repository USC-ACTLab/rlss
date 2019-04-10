#include "spline.h"

template<class T>
class QPSolver {
   public:
   using Vector = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;
   using QPMatrices = typename Spline<T, 3U>::QPMatrices;
   virtual void solve(const QPMatrices& qp, bool& success, Vector& result) = 0;
};