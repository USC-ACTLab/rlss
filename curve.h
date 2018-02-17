#ifndef PATHREPLAN_CURVE_H
#define PATHREPLAN_CURVE_H
#include <vector>
#include "vectoreuc.h"
using namespace std;
class curve {
  /*
    bezier defined between t [0,1]
    use duration to scale
  */
  public:
    /* curve is in this dimension */
    int dimension;
    /* control points of the curve, each have size dimension */
    vector<vectoreuc> cpts;
    /* duration of the curve */
    double duration;

    curve(double dur, int dim);
    void add_cpt(vectoreuc& cpt);
    int size();
    vectoreuc& operator[](int idx);
    void set_duration(double dur);
    vectoreuc eval(double t);
    static int comb(int n, int i);
    double integrate(double from, double to, vector<double>& grad);
    curve& operator-=(const curve& rhs);
};

#endif
