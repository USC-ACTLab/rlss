#ifndef PATHREPLAN_SVMSEPERATOR_H
#define PATHREPLAN_SVMSEPERATOR_H

#include "vectoreuc.h"
#include "hyperplane.h"
#include <vector>
#include "obstacle.h"

using namespace std;

/*
  seperates given points from each of the obstacles.
*/
class SvmSeperator {
  private:
    vector<vectoreuc> pts;
    vector<obstacle2D>* obstacles;

  public:
    SvmSeperator(vector<obstacle2D>* obs);
    void reset_pts();
    void add_pt(const vectoreuc& vec);
    vector<hyperplane> seperate();
    vector<hyperplane> _2_4_seperate();
    vector<hyperplane> _8_4_seperate();
    vector<hyperplane> _32_4_seperate();
};

#endif
