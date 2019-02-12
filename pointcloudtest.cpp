#include "PointCloud.h"
#include "outpututils.hpp"

using namespace ACT;

int main() {
  using VectorDIM = Eigen::Matrix<double, 3U, 1U>;
  PointCloud<double, 3U> pc;
  pc.addPoint(VectorDIM(7,7,7));
  pc.addPoint(VectorDIM(8,9,9));
  pc.addPoint(VectorDIM(6,6,6));
  pc.addPoint(VectorDIM(6.5,8,7));


  pc.convexHull();

  for(const auto& pt: pc._pts) {
    cout << "vc " << pt(0) << " " << pt(1) << " " << pt(2) << endl;
  }

  for(const auto& hp: pc._convexhull) {
    cout << "hp " << hp.normal()(0) << " "
      << hp.normal()(1) << " "<< hp.normal()(2)
      << " " << hp.offset() << endl;
  }

  return 0;
}
