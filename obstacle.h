#ifndef PATHREPLAN_OBSTACLE_H
#define PATHREPLAN_OBSTACLE_H
#include <vector>
#include "vectoreuc.h"

using namespace std;

class obstacle2D {
  public:
    vector<vectoreuc> pts;
    vector<int> ch;
    void add_pt(vectoreuc& pt);
    int size();
    vectoreuc& operator[](int idx);
    static bool comparex(vectoreuc& f, vectoreuc& s);
    bool left_turn(int idx1, int idx2, int idx3);
    void print_current_hull();
    void convex_hull();

};

#endif
