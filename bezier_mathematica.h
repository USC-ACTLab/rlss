#ifndef PATHREPLAN_BEZIERINTGRADMATHEMATICA_H
#define PATHREPLAN_BEZIERINTGRADMATHEMATICA_H

#include <vector>
#include "vectoreuc.h"
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

#endif
