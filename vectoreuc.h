#ifndef PATHREPLAN_VECTOREUC_H
#define PATHREPLAN_VECTOREUC_H
#include <vector>
#include <cmath>
#include <ostream>
#include <bspline.h>
using namespace std;

class vectoreuc {
  public:
    vector<double> crds;
    vectoreuc();
    vectoreuc(int n);
    int size();
    double& operator[](int idx);
    vectoreuc operator-(const vectoreuc& rhs);
    vectoreuc operator-(const splx::Vec& rhs);
    vectoreuc operator+(const vectoreuc& rhs);
    vectoreuc operator/(const double& s);
    vectoreuc normalized();
    double L2norm();
    double dot(vectoreuc& rhs) const;
    friend ostream& operator<<(ostream& out, const vectoreuc& vec);
    void zero();
    vectoreuc operator*(double rhs);

};


#endif
