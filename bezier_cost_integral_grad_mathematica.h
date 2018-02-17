#ifndef PATHREPLAN_BEZIERINTGRADMATHEMATICA_H
#define PATHREPLAN_BEZIERINTGRADMATHEMATICA_H

#include <vector>
#include "vectoreuc.h"
using namespace std;

/*
integrate from a to b
*/
vector<double> bezier_gradient_2d_8pts(double a, double b, vector<vectoreuc>& R);
double bezier_integrate_2d_8pts(double a, double b, vector<vectoreuc>& R);

#endif
