#ifndef PATHREPLAN_HYPERPLANE_H
#define PATHREPLAN_HYPERPLANE_H
#include <vector>
#include "vectoreuc.h"
#include "bspline.h"
#include <Eigen/Dense>
using namespace std;
class hyperplane {
  public:
    vectoreuc normal;
    double distance;
    double dist(vectoreuc& pt);
};
vector<hyperplane> voronoi(vector<Eigen::Matrix<double, 2, 1> >& positions, int robotidx, double robot_radius);
#endif
