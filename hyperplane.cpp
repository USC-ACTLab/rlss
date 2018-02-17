#include "hyperplane.h"

using namespace std;

vector<hyperplane> voronoi(vector<vectoreuc>& positions, int robotidx) {
  vector<hyperplane> res;

  vectoreuc& robotpos = positions[robotidx];

  for(int i=0; i<positions.size(); i++) {
    if(i==robotidx)
      continue;

    hyperplane nplane;
    vectoreuc seperation = positions[i] - robotpos;
    vectoreuc mid = robotpos + (seperation / 2);
    nplane.normal = seperation.normalized();
    nplane.distance = mid.dot(nplane.normal);
    res.push_back(nplane);
  }

  return res;

}
