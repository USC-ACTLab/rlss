#ifndef PATHREPLAN_BEZIERINTGRADMATHEMATICA_H
#define PATHREPLAN_BEZIERINTGRADMATHEMATICA_H

#include <vector>
#include "vectoreuc.h"
#include "hyperplane.h"
using namespace std;

/*
integrate from a to b
*/
vector<double> bezier_gradient_2d_8pts(double a, double b, vector<vectoreuc>& R, double duration);
double bezier_integrate_2d_8pts(double a, double b, vector<vectoreuc>& R, double duration);


/*continuity constraint between the curve defined by control points P, and the curve defined by control points Q
takes euclidean distance between points P^(n)[1] and Q^(n)[0], grad is the gradient of that function wrt to first P and Q (16 vars per curve, 32 vars in total)*/
double bezier_2d_8pts_continuity_euc(vector<double>& P, vector<double>& Q, int n, vector<double>& grad, double durf, double durg);

/*
  what is the distance of bezier curve defined by control points P evaluated at time t from given hyperplane.
  grad is the gradient w.r.t. control points that shows the direction that increases distance most.
*/
double bezier_2d_8pts_distance_from_plane(vector<vectoreuc>& P, hyperplane& plane, vector<double>& grad, double t);

/*
  what is the distance of degreeth derivative of bezier curve with duration dur defined by control points P evaluated at time t from given vector C
  grad is the gradient w.r.t. control points that shows the direction that increases distance most.
*/
double bezier_2d_8pts_ndistance_from_point(vector<double>& P, vectoreuc& C, vector<double>& grad, double t, int degree, double dur);
#endif
