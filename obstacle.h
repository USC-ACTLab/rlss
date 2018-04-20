#ifndef PATHREPLAN_OBSTACLE_H
#define PATHREPLAN_OBSTACLE_H
#include <vector>
#include "vectoreuc.h"
#include "hyperplane.h"

using namespace std;

class obstacle2D {
  public:
    vector<vectoreuc> pts;
    vector<int> ch;
    vector<hyperplane> chplanes;
    void add_pt(vectoreuc& pt);
    int size();
    vectoreuc& operator[](int idx);
    static bool comparex(vectoreuc& f, vectoreuc& s);
    bool left_turn(int idx1, int idx2, int idx3);
    void print_current_hull();
    void convex_hull();
    void ch_planes();
    bool point_inside(vectoreuc& pt);
    double closest_distance(vectoreuc& pt);
};

#endif
