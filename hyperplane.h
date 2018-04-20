#ifndef PATHREPLAN_HYPERPLANE_H
#define PATHREPLAN_HYPERPLANE_H
#include <vector>
#include "vectoreuc.h"
using namespace std;
class hyperplane {
  public:
    vectoreuc normal;
    double distance;
    double dist(vectoreuc& pt);
};
vector<hyperplane> voronoi(vector<vectoreuc>& positions, int robotidx);
#endif
