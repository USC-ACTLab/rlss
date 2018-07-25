#ifndef PATHREPLAN_OCCUPANCYGRID_H
#define PATHREPLAN_OCCUPANCYGRID_H

#include <vector>
#include "obstacle.h"
#include "trajectory.h"
#include "PointCloud.h"

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
    bool occupied(double x, double y, double robot_radius);
    /* returns whether grid[x][y] or neighbors are occupied, which means that grid is occupied in that index */
    bool idx_occupied(index& idx);

    /*checks whether given trajectory passes through an obstacle*/
    bool occupied(trajectory& traj, double robot_radius, double start_time = 0, double end_time = -1);

    void set_occupied(double x, double y);
    void set_free(double x, double y);

    size_t max_i() const;
    size_t max_j() const;

  private:
    vector<vector<bool> > grid;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double ss; // step size
    double ss2; // step size / 2
};

/**
 * Coordinates are assumed to be type T
 * T: float, double or long double
*/
template<typename T>
class OccupancyGrid3D {
  public:
    using GridType = std::vector<std::vector<std::vector<bool> > >;
    using GridSizeType = GridType::size_type;

    enum Direction {
      NONE,
      UP,
      DOWN,
      LEFT,
      RIGHT,
      FORWARD,
      BACK
    };

    /*
     * Encodes a direction information with an index
     * Used for neighbors calculations to indicate which way the neighbors is
     * w.r.t. previous cell
    */
    class DirectionalIndex {
      public:
        GridSizeType i;
        GridSizeType j;
        GridSizeType k;

        Direction d;

        DirectionalIndex(GridSizeType x, GridSizeType y, GridSizeType z, Direction dir): i(x), j(y), k(z), d(dir) {

        }
    };

    OccupancyGrid3D(const std::vector<PointCloud<T, 3> >& obstacles, T stepsize,
      T xmin = -10.0, T xmax = 10.0, T ymin = -10.0, T ymax = 10.0, T zmin = -10.0, T zmax = 10.0):
      _stepsize(stepsize), _xmin(xmin), _xmax(xmax), _ymin(ymin), _ymax(ymax), _zmin(zmin), _zmax(zmax) {
        _grid.resize(std::ceil((_xmax - _xmin) / _stepsize));
        unsigned int xidx = 0;
        for(T x = _xmin; x < _xmax; x += _stepsize, xidx++) {
          _grid[xidx].resize(std::ceil((_ymax - _ymin) / _stepsize));
          unsigned int yidx = 0;
          for(T y = _ymin; y < _ymax; y += _stepsize, yidx++) {
            _grid[xidx][yidx].resize(std::ceil((_zmax - _zmin) / _stepsize), false);
            unsigned int zidx = 0;
            for(T z = _zmin; z < _zmax; z += _stepsize, zidx++) {
              typename PointCloud<T, 3>::Vector cubemin, cubemax;
              cubemin(0) = x;
              cubemin(1) = y;
              cubemin(2) = z;
              cubemax = cubemin + _stepsize;
              typename PointCloud<T, 3>::AlignedBox cube(cubemin, cubemax);
              for(auto obs : obstacles) {
                if(obs.convexHullIntersects(cube)) {
                  _grid[xidx][yidx][zidx] = true;
                  break;
                }
              }
            }
          }
        }
    }

  private:
    GridType _grid;

    /*
     * Boundary of occupancy grid
    */
    T _xmin;
    T _xmax;
    T _ymin;
    T _ymax;
    T _zmin;
    T _zmax;

    /*
     * Step size between cells
    */
    T _stepsize;
};

#endif
