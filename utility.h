#ifndef PATHREPLAN_UTILITY_H
#define PATHREPLAN_UTILITY_H
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include "PointCloud.h"
#include <limits>
#include <queue>

using std::cout;
using std::endl;

using std::vector;
using std::string;


namespace ACT {

template<typename T, unsigned int DIM>
using VectorDIM = Eigen::Matrix<T, DIM, 1U>;

using Eigen::Hyperplane;
using Eigen::Matrix;

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

  T maxProj = std::numeric_limits<T>::lowest();

  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 2; j++) {
      for(int k = 0; k < 2; k++) {
        VectorDIM pt(coords[i](0), coords[j](1), coords[k](2));
        VectorDIM dir = pt - _cntr;
        T proj = dir.dot(normal);
        maxProj = std::max(proj, maxProj);
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


/*
* interpolate from point 'from' to point 'to' with number_of_points points in total.
* number_of_points >= 2.
*/
template<class T, unsigned int DIM>
vector<VectorDIM<T, DIM>, Eigen::aligned_allocator<VectorDIM<T, DIM>>> linearInterpolate(
    const VectorDIM<T, DIM>& from, const VectorDIM<T, DIM>& to,
    unsigned int number_of_points) {
  assert(number_of_points >= 2);
  using VectorDIM = VectorDIM<T, DIM>;
  vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> res;
  res.push_back(from);
  VectorDIM step = (to - from) / (number_of_points - 1);
  VectorDIM current_add = step;
  for(int s = 0; s < number_of_points - 1; s++) {
    res.push_back(from + current_add);
    current_add += step;
  }

  return res;
}

/*
* Divide the segments as best as possible so that total number of segments
* is segment_count
*
* segment i is defined by point pairs corners[i], corners[i+1]
* when corners.size() - 1 < segment_count, try to divide already existing segments
* so that we have segment_count number of segments
*
* do the division in the best way such that
* minimize the length difference between longest segment and the shortest segment
*
* I feel like this can be solved greedily (needs proof!)
*/
template<class T, unsigned int DIM>
vector<VectorDIM<T, DIM>, Eigen::aligned_allocator<VectorDIM<T, DIM> > >
  bestSplitSegments(
  const vector<VectorDIM<T, DIM>, Eigen::aligned_allocator<VectorDIM<T, DIM> > >& corners,
  unsigned int segment_count) {

  using VectorDIM = VectorDIM<T, DIM>;
  using std::priority_queue;

  assert(segment_count > corners.size() - 1);

  class PQSegment {
    public:
      unsigned int start_idx; // segment is from corners[start_idx]  to corners[start_idx + 1]
      unsigned int div_count;
      const vector<VectorDIM, Eigen::aligned_allocator<VectorDIM> > *corners;
      PQSegment(unsigned int fi,
        const vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>>* crnrs):
        start_idx(fi), corners(crnrs), div_count(1) {

      }

      bool operator<(const PQSegment& rhs) const {
        T this_length = ((*corners)[start_idx+1] - (*corners)[start_idx]).norm();
        T rhs_length = ((*corners)[rhs.start_idx + 1] - (*corners)[rhs.start_idx]).norm();
        return (this_length / div_count) < (rhs_length / rhs.div_count);
      }
  };


  priority_queue<PQSegment, vector<PQSegment>> pq;
  for(unsigned int i = 0; i < corners.size()-1; i++) {
    pq.emplace(i, &corners);
  }


  for(unsigned int current_segment_count = corners.size() - 1;
    current_segment_count < segment_count; current_segment_count++) {
    PQSegment top = pq.top();
    top.div_count++;
    pq.pop();
    pq.push(top);
  }


  vector<PQSegment> segments;
  while(!pq.empty()) {
using Eigen::Hyperplane;
    segments.push_back(pq.top());
    pq.pop();
  }

  sort(segments.begin(), segments.end(), [](const PQSegment& lhs, const PQSegment& rhs) {
    return lhs.start_idx < rhs.start_idx;
  });

  vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>> new_corners;

  for(int i = 0; i < segments.size(); i++) {
    const PQSegment& segment = segments[i];
    const VectorDIM& from = corners[segment.start_idx];
    const VectorDIM& to = corners[segment.start_idx + 1];
    unsigned int div_count = segment.div_count;
    auto segment_corners = linearInterpolate<T, DIM>(from , to, div_count + 1);

    for(int j = 0; j < segment_corners.size() - 1; j++) {
      new_corners.push_back(segment_corners[j]);
    }
    if(i == segments.size() - 1) {
      new_corners.push_back(segment_corners.back());
    }
  }

  return new_corners;
}

template<class T, unsigned int DIM>
void voronoi_fix(vector<VectorDIM<T,DIM>,
  Eigen::aligned_allocator<VectorDIM<T, DIM>>>& corners,
  const vector<Hyperplane<T, DIM>>& VORONOI_HYPERPLANES) {
  assert(corners.size() >= 2);
  using VectorDIM = VectorDIM<T, DIM>;
  using Hyperplane = Hyperplane<T, DIM>;


  bool needs_split = false;
  T min_distance = std::numeric_limits<T>::infinity();

  const auto& first_corner = corners[0];
  const auto& second_corner = corners[1];

  VectorDIM split_loc;

  for(const auto& hp: VORONOI_HYPERPLANES) {

    if(hp.signedDistance(second_corner) > 0 && hp.signedDistance(first_corner) < 0) {
      needs_split = true;

      T k = (-hp.offset() - first_corner.dot(hp.normal())) /
        (second_corner.dot(hp.normal()) - first_corner.dot(hp.normal()));
      VectorDIM pt = first_corner + k * (second_corner - first_corner);
      T dist = (pt - first_corner).squaredNorm();
      if(dist < min_distance) {
        min_distance = dist;
        split_loc = pt;
      }
    }
  }

  if(needs_split) {
    split_loc = split_loc - (split_loc - first_corner) * 0.01;
    corners.insert(corners.begin() + 1, split_loc);
  }
}

}

#endif
