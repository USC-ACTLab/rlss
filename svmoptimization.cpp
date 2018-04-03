#include "svmoptimization.h"
#include <vector>
#include "vectoreuc.h"
using namespace std;


double svmoptimization::objective(const vector<double>& x, vector<double>& grad, void* svmoptdata) {
  svmopt_data& svmdata = *((svmopt_data*) svmoptdata);
  vectoreuc& vel = svmdata.velocity_direction;
  double alpha = svmdata.alpha;
  double beta = svmdata.beta;
  if(grad.size() > 0) {
    grad[0] = 2*alpha*x[0] + beta*vel[0];
    grad[1] = 2*alpha*x[1] + beta*vel[1];
    grad[2] = 0;
  }

  return alpha*(x[0]*x[0] + x[1]*x[1])+beta*(x[0]*vel[0] + x[1]*vel[1]);
}


double svmoptimization::obspointconstraint(const vector<double>& x, vector<double>& grad, void* obs_data) {
  svmobs_data& svmdata = *((svmobs_data*) obs_data);
  vectoreuc& pt = *(svmdata.obspt);

  if(grad.size() > 0) {
    grad[0] = -pt[0];
    grad[1] = -pt[1];
    grad[2] = 1;
  }


  return x[2] - pt[0]*x[0] - pt[1]*x[1] + 1;

}


double svmoptimization::robotpointconstraint(const vector<double>& x, vector<double>& grad, void* robot_data) {
  svmrobot_data& svmdata = *((svmrobot_data*) robot_data);
  vectoreuc& pt = *(svmdata.robotpt);

  if(grad.size() > 0) {
    grad[0] = pt[0];
    grad[1] = pt[1];
    grad[2] = -1;
  }

  return pt[0] * x[0] + pt[1] * x[1]  - x[2] + 1;
}
