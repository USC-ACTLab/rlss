#ifndef PATHREPLAN_EDTV2_H
#define PATHREPLAN_EDTV2_H

#include <vector>
#include "obstacle.h"

class edtv2 {
  private:
    double step_size = 0.01;
    double x_max;
    double y_max;
    double x_min;
    double y_min;
    /*
      bottom to top, left to right.
      grid[0][0] = [bottom,left] (y_min,x_min)
      grid[0][1] = (y_min, x_min+step_size)
      grid[1][0] = (y_min+step_size, x_min)
    */
    /*
    positive in inside, negative in outside
    */
    vector<vector<double> > grid;
    double tau; // if distance is less than this no problem, if not (pt - tau)^2 is the cost, (is negative.)

  public:

    edtv2(double ss, double xm, double xM, double ym, double yM, double T);
    void construct(vector<obstacle2D>*);
    double cost(vector<double>& controlpts, double duration, double t, vector<double>& grad);

};

#endif
