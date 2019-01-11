#include "SVM.h"
#include <iostream>
#include <Eigen/StdVector>

using std::cout;
using std::endl;
using namespace ACT;

int main() {
  using AlignedBox = Eigen::AlignedBox<double , 3U>;
  using Hyperplane = Eigen::Hyperplane<double, 3U>;
  using VectorDIM = Eigen::Matrix<double, 3U, 1U>;


  vector<AlignedBox> robot_boxes;
  vector<AlignedBox> obstacle_boxes;

  obstacle_boxes.push_back(AlignedBox(VectorDIM(-0.5, -0.5, -0.5), VectorDIM(0.5, 0.5, 0.5)));
    obstacle_boxes.push_back(AlignedBox(VectorDIM(-10, -10, -10), VectorDIM(-8, -7, -6)));

  robot_boxes.push_back(AlignedBox(VectorDIM(5, 5, -0.5), VectorDIM(6,6,0.5)));
  robot_boxes.push_back(AlignedBox(VectorDIM(5, -0.5, -0.5), VectorDIM(6,0.5,0.5)));

  vector<Hyperplane> res = svm3d(robot_boxes, obstacle_boxes);

  for(auto& b: obstacle_boxes) {
    cout << "o " <<
     b.min()(0) << " " <<
     b.min()(1) << " " <<
     b.min()(2) << " " <<
     b.max()(0) << " " <<
     b.max()(1) << " " <<
     b.max()(2) << endl;
  }

  for(auto& b: robot_boxes) {
    cout << "r " <<
     b.min()(0) << " " <<
     b.min()(1) << " " <<
     b.min()(2) << " " <<
     b.max()(0) << " " <<
     b.max()(1) << " " <<
     b.max()(2) << endl;
  }

  for(auto& hp: res) {
    cout << "h " <<
    hp.normal()(0) << " " <<
    hp.normal()(1) << " " <<
    hp.normal()(2) << " " <<
    hp.offset() << endl;
  }
  return 0;
}
