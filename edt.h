#ifndef PATHREPLAN_EDT_H
#define PATHREPLAN_EDT_H

#include <vector>
#include "obstacle.h"
using namespace std;

class edt {
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
    vector<vector<double> > grid;

  public:

    edt(double ss, double xm, double xM, double ym, double yM);
    void construct(vector<obstacle2D>*);
    double interpolate_for(vector<double>& controlpts, double dur, double t, vector<double>& grad);


};

#endif
