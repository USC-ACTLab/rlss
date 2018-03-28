#include "obstacle.h"
#include <iostream>
#include <algorithm>
using namespace std;


void obstacle2D::add_pt(vectoreuc& pt) {
  pts.push_back(pt);
}
int obstacle2D::size() {
  return pts.size();
}
vectoreuc& obstacle2D::operator[](int idx) {
  return pts[idx];
}

bool obstacle2D::comparex(vectoreuc& f, vectoreuc& s) {
  if(f[0] < s[0]) {
    return true;
  }
  if(f[0] == s[0]) {
    return f[1] < s[1];
  }

  return false;
}

bool obstacle2D::left_turn(int idx1, int idx2, int idx3) {
  vectoreuc dif1(2);
  dif1[0] = pts[ch[idx2]][0] - pts[ch[idx1]][0];
  dif1[1] = pts[ch[idx2]][1] - pts[ch[idx1]][1];
  vectoreuc dif2(2);
  dif2[0] = pts[idx3][0] - pts[ch[idx1]][0];
  dif2[1] = pts[idx3][1] - pts[ch[idx1]][1];

  return dif1[0] * dif2[1] - dif1[1] * dif2[0] >= 0;
}

void obstacle2D::print_current_hull() {
  for(int i=0; i<ch.size(); i++) {
    cout << pts[ch[i]][0] << " " << pts[ch[i]][1] << endl;
  }
  cout << endl;
}

void obstacle2D::convex_hull() {
  ch.clear();
  sort(pts.begin(), pts.end(), comparex);
  ch.push_back(0);
  ch.push_back(1);
  for(int i=2; i<pts.size(); i++) {
    int last_idx = ch.size() - 1;
    while(ch.size() > 1 && left_turn(last_idx-1, last_idx, i)) {
      ch.pop_back();
      last_idx--;
    }
    ch.push_back(i);
  }

  int min_size = ch.size();
  ch.push_back(pts.size()-2);
  for(int i=pts.size()-3; i>=0; i--) {
    int last_idx = ch.size()-1;
    while(ch.size() > min_size && left_turn(last_idx-1, last_idx, i)) {
      ch.pop_back();
      last_idx--;
    }
    ch.push_back(i);
  }
}

void obstacle2D::ch_planes() {
  chplanes.clear();
  for(int i=0; i<ch.size()-1; i++) {
    hyperplane hp;
    vectoreuc& firstpt = pts[ch[i]];
    vectoreuc& secondpt = pts[ch[i+1]];
    vectoreuc normal(2);
    normal[0] = secondpt[1] - firstpt[1];
    normal[1] = firstpt[0] - secondpt[0];
    hp.normal = normal.normalized();
    hp.distance = firstpt.dot(hp.normal);
    chplanes.push_back(hp);
  }
}
