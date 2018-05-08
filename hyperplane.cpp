#include "hyperplane.h"
#include <cmath>
using namespace std;

#define PI 3.14159265

vector<hyperplane> voronoi(vector<vectoreuc>& positions, int robotidx, double robot_radius) {
  static double COS = cos(15*PI/180);
  static double SIN = sin(15*PI/180);

  vector<hyperplane> res;

  vectoreuc& robotpos = positions[robotidx];

  for(int i=0; i<positions.size(); i++) {
    if(i==robotidx)
      continue;

    hyperplane nplane;
    vectoreuc seperation = positions[i] - robotpos;
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
