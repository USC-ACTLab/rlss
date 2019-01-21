#ifndef PATHREPLAN_UTILITY_H
#define PATHREPLAN_UTILITY_H
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include "PointCloud.h"
#include <limits>


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
* If you want aligned box to be in the negative side of the hyperplane
* how much should you shift the hyperplane in the opposite direction of its normal
* so that when you constrain the center of the aligned box to be in the negative side
* of the resulting hyperplane, you would effectively be constraining the all aligned box
* to be in the negative side of the original hyperplane?
*
* In short: shift the svm and voronoi hyperplanes for a given robot box
*/
template<class T>
void shiftHyperplane(const Eigen::AlignedBox<T, 3U>& box, Eigen::Hyperplane<T, 3U>& hp) {
  using VectorDIM = Eigen::Matrix<T, 3U, 1U>;
  const VectorDIM& _min = box.min();
  const VectorDIM& _max = box.max();
  const vector<VectorDIM, Eigen::aligned_allocator<VectorDIM> > coords{_min, _max};
  const VectorDIM& _cntr = (_min + _max) / 2;
  const VectorDIM& normal = hp.normal();
  VectorDIM dir;
  T proj;

  T maxProj = numeric_limits<T>::lowest();

  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 2; j++) {
      for(int k = 0; k < 2; k++) {
        VectorDIM pt(coords[i](0), coords[j](1), coords[k](2));
        VectorDIM dir = pt - _cntr;
        T proj = dir.dot(normal);
        maxProj = max(proj, maxProj);
      }
    }
  }

  hp.offset() += maxProj;
}


/*
* STATES[x][y] gives the yth derivative of xth robot
*
* Calculates the BUFFERED voronoi cell for robot r
*/
template<class T, unsigned int DIM>
std::vector<Eigen::Hyperplane<T, DIM> > voronoi(std::vector<std::vector<Eigen::Matrix<T, DIM, 1>
          , Eigen::aligned_allocator<Eigen::Matrix<T, DIM, 1> > > >& STATES, unsigned int r, T robot_radius) {
  using VectorDIM = Eigen::Matrix<T, DIM, 1>;
  using Hyperplane = Eigen::Hyperplane<T, DIM>;
  using AlignedBox = Eigen::AlignedBox<T, DIM>;

  vector<Hyperplane> hyperplanes;
  VectorDIM rPos = STATES[r][0];
  VectorDIM rad(robot_radius, robot_radius, robot_radius);
  AlignedBox rbox(rPos - rad, rPos + rad);
  for(int i = 0; i < STATES.size(); i++) {
    if(i != r) {
      const VectorDIM& iPos = STATES[i][0];
      VectorDIM normal = (iPos - rPos).normalized();
      VectorDIM mid = (iPos + rPos) / 2;
      double D = -mid.dot(normal);
      Hyperplane hp(normal, D);
      shiftHyperplane(rbox, hp);
      hyperplanes.push_back(hp);
    }
  }
  return hyperplanes;
}



}

#endif
