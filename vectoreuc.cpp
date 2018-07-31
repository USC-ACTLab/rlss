#include "vectoreuc.h"
#include <vector>
using namespace std;

vectoreuc::vectoreuc() {}

vectoreuc::vectoreuc(int n) {
  crds.resize(n);
}

int vectoreuc::size() {
  return crds.size();
}

double& vectoreuc::operator[](int idx) {
  return crds[idx];
}

vectoreuc vectoreuc::operator-(const vectoreuc& rhs) {
  vectoreuc res(crds.size());
  for(int i=0; i<crds.size(); i++) {
    res[i] = crds[i] - rhs.crds[i];
  }
  return res;
}

vectoreuc vectoreuc::operator+(const vectoreuc& rhs) {
  vectoreuc res(crds.size());
  for(int i=0; i<crds.size(); i++) {
    res[i] = crds[i] + rhs.crds[i];
  }
  return res;
}

vectoreuc vectoreuc::operator/(const double& s) {
  vectoreuc res(crds.size());
  for(int i=0; i<crds.size(); i++) {
    res[i] = crds[i] / s;
  }
  return res;
}


vectoreuc vectoreuc::normalized() {
  vectoreuc res(crds.size());
  double length = 0;
  for(int i=0; i<crds.size(); i++) {
    length += crds[i] * crds[i];
  }
  length = sqrt(length);
  for(int i=0; i<crds.size(); i++) {
    res[i] = crds[i] / length;
  }
  return res;
}

double vectoreuc::dot(vectoreuc& rhs) const {
  double res = 0;
  for(int i=0; i<crds.size(); i++) {
    res += crds[i] * rhs[i];
  }
  return res;
}


ostream& operator<<(ostream& out, const vectoreuc& vec) {
  out << "(";
  for(int i=0; i<vec.crds.size()-1; i++) {
    out << vec.crds[i] << ",";
  }
  out << vec.crds[vec.crds.size()-1] <<  ")";
  return out;
}

double vectoreuc::L2norm() {
  double length = 0;
  for(int i=0; i<crds.size(); i++) {
    length += crds[i] * crds[i];
  }
  return sqrt(length);
}

void vectoreuc::zero() {
  for(int i=0; i<crds.size(); i++)
    crds[i] = 0;
}


vectoreuc vectoreuc::operator*(double rhs) {
  vectoreuc res(crds.size());
  for(int i=0; i<res.size(); i++) {
    res[i] = rhs * crds[i];
  }
  return res;
}


vectoreuc vectoreuc::operator-(const Eigen::Matrix<double, Eigen::Dynamic, 1>& rhs) {
  vectoreuc res(crds.size());
  for(int i = 0; i < res.size(); i++)
    res[i] = crds[i] - rhs(i);
  return res;
}
