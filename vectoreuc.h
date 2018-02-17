#ifndef PATHREPLAN_VECTOREUC_H
#define PATHREPLAN_VECTOREUC_H
#include <vector>
#include <cmath>
using namespace std;

class vectoreuc {
  public:
    vector<double> crds;
    vectoreuc();
    vectoreuc(int n);
    int size();
    double& operator[](int idx);
    vectoreuc operator-(const vectoreuc& rhs);
    vectoreuc operator+(const vectoreuc& rhs);
    vectoreuc operator/(const double& s);
    vectoreuc normalized();
    double dot(vectoreuc& rhs);
};
#endif
