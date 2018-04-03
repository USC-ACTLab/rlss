#ifndef PATHREPLAN_SVMOPTIMIZATION_H
#define PATHREPLAN_SVMOPTIMIZATION_H
#include "vectoreuc.h"
#include <vector>
using namespace std;


class svmopt_data {
  public:
    vectoreuc velocity_direction;
    double alpha;
    double beta;
    /*
      alpha * |w| + beta * w (dot) velocity_direction
    */
};

class svmobs_data {
  public:
    vectoreuc* obspt;
};

class svmrobot_data {
  public:
    vectoreuc* robotpt;
};

class svmoptimization {
  public:
    static double objective(const vector<double>& x, vector<double>& grad, void* f_data);
    static double obspointconstraint(const vector<double>& x, vector<double>& grad, void* o_data);
    static double robotpointconstraint(const vector<double>& x, vector<double>& grad, void* r_data);
};

#endif
