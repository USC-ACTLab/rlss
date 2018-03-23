#ifndef PATHREPLAN_TRAJECTORY_H
#define PATHREPLAN_TRAJECTORY_H
#include <vector>
#include "curve.h"
#include <cassert>
using namespace std;
class trajectory {
  public:
      vector<curve> curves;
      void add_curve(curve& crv);
      curve& operator[](int idx);
      int size();
      vectoreuc eval(double t);
      /* eval at given t, but add current curve index to curveidx param, and corresponding time in curve to ct normalized to [0,1] interval from [0, duration] interval */
      vectoreuc eval(double t, int& curveidx, double& ct);
      /*
        evaluate nth derivative of trajectory
      */
      vectoreuc neval(double t, int n);
      double integrate(double from, double to, vector<double>& grad);
      trajectory& operator-=(const trajectory& rhs);
};
#endif
