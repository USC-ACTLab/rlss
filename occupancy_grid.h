#ifndef PATHREPLAN_OCCUPANCYGRID_H
#define PATHREPLAN_OCCUPANCYGRID_H

#include <vector>
#include "obstacle.h"
#include "trajectory.h"

using namespace std;

class OG {
  public:
    enum direction {
      NONE,
      UP,
      DOWN,
      LEFT,
      RIGHT
    };

    class index {
      public:
        int i;
        int j;
        direction d;


        index(int a, int b);
        index(int a, int b, direction c);
    };
    OG(double step_size, double xm, double xM, double ym, double yM, vector<obstacle2D>& obstacles);

    /* returns the non-occupied neighbors of given grid index with direction info */
    vector<index> neighbors(index& cur);

    /* returns indices of block that given coordinate inside of */
    index get_index(double x, double y);

    /* get coordinates of the middle point of given grid location */
    /* returns pair<double,double>( x, y ) */
    pair<double, double> get_coordinates(index& idx);

    /* returns whether given point in space is occupied */
    bool occupied(double x, double y);
    /* returns whether grid[x][y] or neighbors are occupied, which means that grid is occupied in that index */
    bool idx_occupied(index& idx);

    /*checks whether given trajectory passes through an obstacle*/
    bool occupied(trajectory& traj);
  private:
    vector<vector<bool> > grid;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double ss; // step size
    double ss2; // step size / 2



};

#endif
