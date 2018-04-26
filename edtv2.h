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

  public:

    edtv2(double ss, double xm, double xM, double ym, double yM);
    void construct(vector<obstacle2D>*);
    double interpolate_for(vector<double>& controlpts, double dur, double t, vector<double>& grad);

};

#endif
