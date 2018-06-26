#include "hyperplane.h"
#include <cmath>
using namespace std;

#define PI 3.14159265

vector<hyperplane> voronoi(vector<splx::Vec>& positions, int robotidx, double robot_radius) {
  static double COS = cos(15*PI/180);
  static double SIN = sin(15*PI/180);

  vector<vectoreuc> poss;
  for(unsigned int i = 0; i<positions.size(); i++) {
    vectoreuc pos(2);
    pos[0] = positions[i](0);
    pos[1] = positions[i](1);
    poss.push_back(pos);
  }

  vector<hyperplane> res;

  vectoreuc& robotpos = poss[robotidx];

  for(int i=0; i<poss.size(); i++) {
    if(i==robotidx)
      continue;

    hyperplane nplane;
    vectoreuc seperation = poss[i] - robotpos;
    vectoreuc mid = robotpos + (seperation / 2);
    seperation = seperation.normalized();

    /*seperation[0] = COS * seperation[0] - SIN * seperation[1];
    seperation[1] = SIN * seperation[0] + COS * seperation[1];*/
    nplane.normal = seperation.normalized();
    nplane.distance = mid.dot(nplane.normal) - robot_radius;
    res.push_back(nplane);
  }

  return res;

}

double hyperplane::dist(vectoreuc& pt) {
  return normal.dot(pt) - distance;
}
