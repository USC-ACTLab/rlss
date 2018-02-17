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
      double integrate(double from, double to, vector<double>& grad);
      trajectory& operator-=(const trajectory& rhs);
};
#endif
