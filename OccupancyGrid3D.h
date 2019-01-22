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
        _grid.resize(std::ceil((_xmax - _xmin) / _stepsize));
        unsigned int xidx = 0;
        for(T x = _xmin; x < _xmax; x += _stepsize, xidx++) {
          _grid[xidx].resize(std::ceil((_ymax - _ymin) / _stepsize));
          unsigned int yidx = 0;
          for(T y = _ymin; y < _ymax; y += _stepsize, yidx++) {
            _grid[xidx][yidx].resize(std::ceil((_zmax - _zmin) / _stepsize), false);
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
            if(isOccupied(idx)) {
              return true;
            }
          }
        }
      }
      return false;
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
    vector<pair<Index, unsigned int>> addRobot(const VectorDIM& pos, T r) {
      VectorDIM rad(r, r, r);
      VectorDIM _min = pos - rad;
      VectorDIM _max = pos + rad;
      auto minidx = getIndex(_min(0), _min(1), _min(2));
      auto maxidx = getIndex(_max(0), _max(1), _max(2));

      vector<pair<Index, unsigned int>> changes;

      VectorDIM ss2(_stepsize / 2, _stepsize / 2, _stepsize / 2);

      for(auto i = minidx.i; i <= maxidx.i; i++) {
        for(auto j = minidx.j; j <= maxidx.j; j++) {
          for(auto k = minidx.k; k <= maxidx.k; k++) {
            if(_grid[i][j][k] == false) {
              Index idx(i, j, k);
              VectorDIM coord = getCoordinates(idx);
              VectorDIM topleft(coord - ss2);
              VectorDIM bottomright(coord + ss2);
              AlignedBox cube(topleft, bottomright);

              _occupied_boxes.push_back(cube);
              _grid[i][j][k] = true;

              changes.push_back(make_pair(idx,
                 (unsigned int) _occupied_boxes.size() - 1));
            }
          }
        }
      }

      return changes;
    }

    /*
    * Undoes the changes done by addRobot function
    *
    * Changes should be sorted in increasing order
    * by their second members (i.e. occupied box indexes)
    */
    void undoRobotChanges(const vector<pair<Index, unsigned int>>& changes) {
      for(auto it = changes.rbegin(); it != changes.rend(); it++) {
        const auto& change = *it;
        const auto& grididx = change.first;
        const unsigned int boxidx = change.second;

        _grid[grididx.i][grididx.j][grididx.k] = false;
        if(boxidx == _occupied_boxes.size() - 1) {
          _occupied_boxes.pop_back();
        } else {
          auto loc = _occupied_boxes.begin();
          advance(loc, boxidx);
          _occupied_boxes.erase(loc);
        }
      }
    }

    std::vector<AlignedBox> _occupied_boxes;

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
