#include "edt.h"

edt::edt(double ss, double xm, double xM, double ym, double yM): step_size(ss), x_min(xm), x_max(xM), y_min(ym), y_max(yM) {

}


void edt::construct(vector<edt::line_segment*> segments) {
  int xcnt = (x_max-x_min) / step_size + 0.5;
  int ycnt = (y_max-y_min) / step_size + 0.5;

  for(double y = y_min; y<=y_max; y+=step_size) {
    vector<double> dist;
    for(double x = x_min; x<=x_max; x+=step_size) {

    }
    grid.push_back(dist);
  }
}
