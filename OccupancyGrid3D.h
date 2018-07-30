#ifndef ACT_OCCUPANCYGRID3D_H
#define ACT_OCCUPANCYGRID3D_H

#include <vector>
#include <iostream>
#inclide <tuple>

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

        Index(GridSizeType x, GridSizeType y, GridSizeType z): i(x), j(y), k(z) {

        }

        Index(const Index& rhs): i(rhs.i), j(rhs.j), k(rhs,k) {

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

    }


    enum Direction {
      NONE,
      XP, // x++
      XM, // x--
      YP, // y++
      YM, // y--
      ZP, // z++
      ZM  // z--
    };
    /*
     * Encodes a direction information with an index
     * Used for neighbors calculations to indicate which way the neighbors is
     * w.r.t. previous cell
    */
    class DirectionalIndex {
      public:
        Index index;
        Direction d;

        DirectionalIndex(GridSizeType x, GridSizeType y, GridSizeType z, Direction dir): index(x, y, z), d(dir) {

        }


        DirectionalIndex(const Index& idx, Direction dir): index(idx), d(dir) {

        }

        GridSizeType& operator[](unsigned int idx) {
          return index[idx];
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
              typename ACT::PointCloud<T, 3>::Vector cubemin, cubemax;
              cubemin(0) = x;
              cubemin(1) = y;
              cubemin(2) = z;
              cubemax = cubemin + _stepsize;
              typename ACT::PointCloud<T, 3>::AlignedBox cube(cubemin, cubemax);
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

    /**
     * Returns the index of the grid that x, y, z is inside of.
    */
    inline Index getIndex(double x, double y, double z) const {
      Index idx;
      idx.i = (x - _xmin) / _stepsize;
      idx.j = (y - _ymin) / _stepsize;
      idx.k = (z - _zmin) / _stepsize;

      if(!insideGrid(idx)) {
        throw OutOfBoundsException("coordinate out of bounds");
      }

      return idx;
    }

    /*
     * Returns the neighbors of idx which are inside the grid with direction
     * information attached to each neighbor.
    */
    std::vector<DirectionalIndex> getNeighbors(const Index& idx) const {
      const static char** D = {{0,0,1,Direction.ZP}, {0,0,-1,Direction.ZM},
                               {0,1,0,Direction.YP}, {0,-1,0,Direction.YM},
                               {1,0,0,Direction.XP}, {-1,0,0,Direction.XM}};
      std::vector<DirectionalIndex> N;
      for(unsigned char i = 0; i < 6; i++) {
        Index index;
        index.i = idx + D[i][0];
        index.j = idx + D[i][1];
        index.k = idx + D[i][2];
        if(insideGrid(index)) {
          N.push_back(DirectionalIndex(index, D[i][3]));
        }
      }
      return N;
    }


    /*
     * Returns the center of the given cell index
    */
    std::tuple<double, double, double> getCoordinates(const Index& idx) const {
      if(!insideGrid(idx)) {
        throw OutOfBoundsException("index out of bounds");
      }

      const double x = _xmin + _stepsize * idx[0] + _stepsize / 2;
      const double y = _ymin + _stepsize * idx[1] + _stepsize / 2;
      const double z = _zmin + _stepsize * idx[2] + _stepsize / 2;
      return std::make_tuple(x, y, z);
    }


    bool isOccupied(const Index& idx) const {
      if(!insideGrid(idx)) {
        throw OutOfBoundsException("coordinate out of bounds");
      }

      return _grid[idx.i][idx.j][idx.k];
    }

    bool isOccupied(double x, double y, double z) {
      Index idx = getIndex(x, y, z);
      return _grid[idx.i][idx.j][idx.k];
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
