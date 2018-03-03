#ifndef PATHREPLAN_OPTIMIZATION_H
#define PATHREPLAN_OPTIMIZATION_H
#include "trajectory.h"
#include "obstacle.h"
#include <vector>
#include "hyperplane.h"

using namespace std;


class problem_data {
  public:
    double current_t;
    double delta_t;
    trajectory* original_trajectory;
    int problem_dimension;
};

/*
voronoi constraint for one point only.
*/
class voronoi_data {
  public:
    problem_data* pdata;
    hyperplane plane;
    int curve_idx;
    int point_idx;
};

class obstacle_data {
  public:
    problem_data* pdata;
    obstacle2D* obs;
};

class continuity_data {
  public:
    int n; // continuity degree
    int pd; // problem dimension
    int c1; // curve 1 index
    int c2; // curve 2 index
};

class optimization {
  public:
    static double objective(const vector<double>& x, vector<double>& grad, void* f_data);
    static double voronoi_constraint(const vector<double>& x, vector<double>& grad, void* v_data);
    static double obstacle_constraint(const vector<double>& x, vector<double>& grad, void* o_data);
    static double continuity_constraint(const vector<double>& x, vector<double>& grad, void* c_data);
};


#endif
