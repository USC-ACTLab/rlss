#include <vector>
#include "trajectory.h"
#include <iostream>

using namespace std;


void trajectory::add_curve(curve& crv) {
  curves.push_back(crv);
}

curve& trajectory::operator[](int idx) {
  return curves[idx];
}

int trajectory::size() {
  return curves.size();
}

vectoreuc trajectory::eval(double t) {
  int i = 0;
  while(i<curves.size() && curves[i].duration < t) {
    t -= curves[i].duration;
    i++;
  }
  if(i==curves.size())
    return curves[i-1].eval(curves[i-1].duration);
  return curves[i].eval(t);
}

double trajectory::integrate(double from, double to, vector<double>& grad) {
  int curveidx;
  int gradidx = 0;
  for(curveidx = 0; curveidx<curves.size(); curveidx++) {
    if(from >= curves[curveidx].duration) {
      from -= curves[curveidx].duration;
      to -= curves[curveidx].duration;
      if(grad.size() > 0) {
        for(int i=0; i<curves[curveidx].size(); i++) {
          for(int j=0; j<curves[curveidx].dimension; j++) {
            grad[gradidx++] = 0;
          }
        }
      }
    } else {
      break;
    }
  }

  from = max(from, 0.0); // just to reset potential underflows.


  double res = 0;

  for(; to > 0 && curveidx < curves.size(); to-=curves[curveidx].duration, curveidx++) {
    vector<double> grdient(curves[curveidx].size() * curves[curveidx][0].size());
    res += curves[curveidx].integrate(from, min(to, curves[curveidx].duration), grdient);
    if(grad.size() > 0) {
      for(int i=0; i<grdient.size(); i++) {
        grad[gradidx++] = grdient[i];
      }
    }
    from = 0; // basically resets the first from value.
  }

  return res;
}

trajectory& trajectory::operator-=(const trajectory& rhs) {
  assert(curves.size() == rhs.curves.size());

  for(int i=0; i<size(); i++) {
    curves[i] -= rhs.curves[i];
  }

  return *this;
}
