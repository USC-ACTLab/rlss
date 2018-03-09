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

/*
continuity constraint between curve defined by control points P, and the curve defined by control points Q
updates results pointer with P^(n)[1] - Q^(n)[0] with gradient a vector of gradients for each dimension.
each gradient is a gradient with respect to variables in first P then Q.
check mathematica/continuity.nb
*/
void bezier_2d_8pts_continuity(const double* P, const double* Q, int n, vector<vector<double> >& grad, double* result);


/*continuity constraint between the curve defined by control points P, and the curve defined by control points Q
takes euclidean distance between points P^(n)[1] and Q^(n)[0], grad is the gradient of that function wrt to first P and Q (16 vars per curve, 32 vars in total)*/
double bezier_2d_8pts_continuity_euc(vector<double>& P, vector<double>& Q, int n, vector<double>& grad);

/*
  what is the distance of bezier curve defined by control points P evaluated at time t from given hyperplane.
  grad is the gradient w.r.t. control points that shows the direction that increases distance most.
*/
double bezier_2d_8pts_distance_from_plane(vector<vectoreuc>& P, hyperplane& plane, vector<double>& grad, double t);
#endif
