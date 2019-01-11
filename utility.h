#ifndef PATHREPLAN_UTILITY_H
#define PATHREPLAN_UTILITY_H
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include "PointCloud.h"


using std::cout;
using std::endl;

using std::vector;
using std::string;

namespace ACT {

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

double linearInterpolation(double start, double end, size_t idx, size_t count)
{
  return start + (end - start) * idx / (count - 1);
}

// trim from start
static inline std::string &ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            std::not1(std::ptr_fun<int, int>(std::isspace))));
    return s;
}

// trim from end
static inline std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
            std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
    return ltrim(rtrim(s));
}

static inline std::vector<std::string> tokenize(const std::string& inp, char delim) {
  vector<string> tokens;
  string token;
  for(int i = 0; i<inp.size(); i++) {
    if(inp[i] == delim) {
      tokens.push_back(token);
      token = "";
    } else {
      token.push_back(inp[i]);
    }
  }
  tokens.push_back(token);
  return tokens;
}


/*
  STATES[x][y] gives the yth derivative of xth robot
*/
template<class T, unsigned int DIM>
std::vector<Eigen::Hyperplane<T, DIM> > voronoi(std::vector<std::vector<Eigen::Matrix<T, DIM, 1>
          , Eigen::aligned_allocator<Eigen::Matrix<T, DIM, 1> > > >& STATES, unsigned int r) {
  using VectorDIM = Eigen::Matrix<T, DIM, 1>;
  using Hyperplane = Eigen::Hyperplane<T, DIM>;

  vector<Hyperplane> hyperplanes;
  VectorDIM rPos = STATES[r][0];
  for(int i = 0; i < STATES.size(); i++) {
    if(i != r) {
      const VectorDIM& iPos = STATES[i][0];
      VectorDIM normal = (iPos - rPos).normalized();
      VectorDIM mid = (iPos + rPos) / 2;
      double D = mid.dot(normal);
      Hyperplane hp(normal, -D);
      hyperplanes.push_back(hp);
    }
  }
  return hyperplanes;
}

}

#endif
