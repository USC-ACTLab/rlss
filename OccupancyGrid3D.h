#ifndef ACT_OCCUPANCYGRID3D_H
#define ACT_OCCUPANCYGRID3D_H

#include <Eigen/StdVector>
#include <iostream>
#include <tuple>
#include "spline.h"
#include <Eigen/Dense>
#include <algorithm>
#include <iterator>

using std::vector;
using std::pair;
using std::numeric_limits;
using std::min;
using std::max;
using std::make_pair;
using std::advance;

namespace ACT {

/**
 * Coordinates are assumed to be type T
 * T: float, double or long double
*/
template<typename T>
class OccupancyGrid3D {
  public:
    using GridType = std::vector<std::vector<std::vector<bool> > >;
    using GridSizeType = GridType::size_type;
    using VectorDIM = Eigen::Matrix<T, 3U, 1U>;
    using AlignedBox = Eigen::AlignedBox<T, 3U>;

    class OutOfBoundsException : public std::runtime_error {
      public:
        OutOfBoundsException(const char * msg) : std::runtime_error(msg) {

        }
    };


    /*
     * Index in a grid.
     * _grid[i][j][k] is the occupancy information
    */
    class Index {
      public:
        GridSizeType i;
        GridSizeType j;
        GridSizeType k;

        Index() {

        }

        Index(GridSizeType x, GridSizeType y, GridSizeType z): i(x), j(y), k(z) {

        }

        Index(const Index& rhs): i(rhs.i), j(rhs.j), k(rhs.k) {

        }

        GridSizeType& operator[](unsigned int idx) {
          switch(idx) {
            case 0U:
              return i;
            case 1U:
              return j;
            case 2U:
              return k;
            default:
              throw OutOfBoundsException("index out of bounds");
          }
        }

    };



    OccupancyGrid3D(const std::vector<ACT::PointCloud<T, 3> >& obstacles, T stepsize,
      T xmin = -10.0, T xmax = 10.0, T ymin = -10.0, T ymax = 10.0, T zmin = -10.0, T zmax = 10.0):
      _stepsize(stepsize), _xmin(xmin), _xmax(xmax), _ymin(ymin), _ymax(ymax), _zmin(zmin), _zmax(zmax) {
        _grid.resize(std::ceil((_xmax - _xmin) / _stepsize + 1));
        unsigned int xidx = 0;
        for(T x = _xmin; x < _xmax; x += _stepsize, xidx++) {
          _grid[xidx].resize(std::ceil((_ymax - _ymin) / _stepsize + 1));
          unsigned int yidx = 0;
          for(T y = _ymin; y < _ymax; y += _stepsize, yidx++) {
            //std::cout << yidx << " " << _grid[xidx].size() << std::endl;
            _grid[xidx][yidx].resize(std::ceil((_zmax - _zmin) / _stepsize + 1), false);
            unsigned int zidx = 0;
            for(T z = _zmin; z < _zmax; z += _stepsize, zidx++) {
              typename ACT::PointCloud<T, 3>::VectorDIM cubemin, cubemax;
              cubemin(0) = x;
              cubemin(1) = y;
              cubemin(2) = z;
              cubemax(0) = cubemin(0) + _stepsize;
              cubemax(1) = cubemin(1) + _stepsize;
              cubemax(2) = cubemin(2) + _stepsize;
              AlignedBox cube(cubemin, cubemax);
              for(auto obs : obstacles) {
                if(obs.convexHullIntersects(cube)) {
                  _occupied_boxes.push_back(cube);
                  //std::cout << xidx << " " << yidx << " " << zidx << std::endl;
                  _grid[xidx][yidx][zidx] = true;
                  break;
                }
              }
            }
          }
        }
    }

    /**
     * Returns the index of the grid that x, y, z is inside of.
    */
    inline Index getIndex(T x, T y, T z) const {
      Index idx;
      idx.i = (x - _xmin) / _stepsize;
      idx.j = (y - _ymin) / _stepsize;
      idx.k = (z - _zmin) / _stepsize;

      if(!insideGrid(idx)) {

        throw OutOfBoundsException("coordinate out of bounds");
      }

      return idx;
    }

    inline Index getIndex(const VectorDIM& pos) const {
      return getIndex(pos(0), pos(1), pos(2));
    }


    /*
     * Returns the center of the given cell index
    */
    VectorDIM getCoordinates(const Index& idx) const {
      if(!insideGrid(idx)) {
        throw OutOfBoundsException("index out of bounds");
      }

      const T x = _xmin + _stepsize * idx.i + _stepsize / 2;
      const T y = _ymin + _stepsize * idx.j + _stepsize / 2;
      const T z = _zmin + _stepsize * idx.k + _stepsize / 2;
      return VectorDIM(x, y, z);
    }

    VectorDIM getCoordinates(unsigned int x, unsigned int y, unsigned int z) {
      return getCoordinates(Index(x, y, z));
    }

    bool isOccupied(const Index& idx) const {
      if(!insideGrid(idx)) {
        throw OutOfBoundsException("coordinate out of bounds");
      }

      return _grid[idx.i][idx.j][idx.k];
    }

    bool isOccupied(T x, T y, T z) const {
      Index idx = getIndex(x, y, z);
      return _grid[idx.i][idx.j][idx.k];
    }

    /*
     * Check if the occupancy grid is occupied at point x,y,z so that when a robot
     * with radius r stays at x,y,z no cells that touches to robot are occupied
    */
    bool isOccupied(T x, T y, T z, T r) const {
      Index idxmin = getIndex(x-r, y-r, z-r);
      Index idxmax = getIndex(x+r, y+r, z+r);
      for(GridSizeType i = idxmin.i; i<=idxmax.i; i++) {
        for(GridSizeType j = idxmin.j; j<=idxmax.j; j++) {
          for(GridSizeType k = idxmin.k; k<=idxmax.k; k++) {
            Index idx(i, j, k);
            try {
              if(isOccupied(idx)) {
                return true;
              }
            } catch(const OutOfBoundsException& exp) {

            }
          }
        }
      }
      return false;
    }

    /*
     * Check if the occupancy grid is occupied at point x,y,z so that when a robot
     * with radius r stays at x,y,z no cells that touches to robot are occupied
    */
    bool isOccupied(const Index& idx, T r) const {
      auto coord = getCoordinates(idx);
      return isOccupied(coord[0], coord[1], coord[2], r);
    }

    /*
     * Check if spl hits any obstacle when u\in[startTime, endTime] when a robot
     * with radius r traverses it.
    */
    bool isOccupied(const splx::Spline<T, 3>& spl, T r, T startTime, T endTime) const {
      using namespace splx;
      const T dt = 0.01;
      typename splx::Spline<T, 3>::VectorDIM vec;
      for(T t = startTime; t <= endTime; t += dt) {
        vec = spl.eval(t, 0);
        if(isOccupied(vec(0), vec(1), vec(2), r)) {
          return true;
        }
      }
      return false;
    }

    GridSizeType iMax() const {
      return _grid.size();
    }

    GridSizeType jMax() const {
      assert(_grid.size() > 0);
      return _grid[0].size();
    }

    GridSizeType kMax() const {
      assert(_grid.size() > 0);
      assert(_grid[0].size() > 0);
      return _grid[0][0].size();
    }

    /*
    * Adds a robot to occupancy grid positioned at pos with cube side 2 * r
    *
    * Returns the indexes that are set true in _grid,
    * and indexes that are added in occupied_boxes
    */
    void addOtherRobot(const VectorDIM& pos, T r) {
      VectorDIM rad(r, r, r);
      VectorDIM _min = pos - rad;
      VectorDIM _max = pos + rad;
      _other_robot_boxes.push_back(AlignedBox(_min, _max));
    }

    void clearOtherRobots() {
      _other_robot_boxes.clear();
    }

    std::vector<AlignedBox> _occupied_boxes;
    std::vector<AlignedBox> _other_robot_boxes;

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


    /**
     * Returns true if the given index is exists in _grid array
    */
    inline bool insideGrid(const Index& idx) const {
      return idx.i < _grid.size()
        && _grid.size() != 0 && idx.j < _grid[0].size()
        && _grid[0].size() != 0 && idx.k < _grid[0][0].size();
    }


};

}

#endif
