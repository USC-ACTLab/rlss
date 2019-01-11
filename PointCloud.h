#ifndef ACT_POINTCLOUD_H
#define ACT_POINTCLOUD_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>
#include <Eigen/StdVector>

#include <iostream>

namespace ACT {

/**
 * T will be float, double or long double presumably.
 *
 * DIM is the dimension points are in.
 *
 * This base function is actually  the point cloud. Added for PointCloud class
 * to be specifically initalized for some template variables
 *
*/
template <typename T, unsigned int DIM>
class PointCloudBase {

  public:
    using VectorDIM = Eigen::Matrix<T, DIM, 1>;
    using Hyperplane = Eigen::Hyperplane<T, DIM>;
    using MatrixDIM = Eigen::Matrix<T, DIM, DIM>;
    using AlignedBox = Eigen::AlignedBox<T, DIM>;



    class LinearDependenceException : public std::runtime_error {
      public:
        LinearDependenceException(const char * msg) : std::runtime_error(msg) {

        }
    };

    /*
     * Point cloud
    */
    std::vector<VectorDIM, Eigen::aligned_allocator<VectorDIM> > _pts;


    /*
     * Hyperplanes of the convex hull where normals are pointing inside the convex hull.
    */
    std::vector<Hyperplane, Eigen::aligned_allocator<Hyperplane> > _convexhull;

    /*
     * Indices of points from which convex hull hyperplanes are computed.
     *
     * Each vector in this vector corresponds to the hyperplane in the same index
     * in _convexhull vector. Therefore each element of this vector is a vector of
     * DIM elements.
     *
    */
    std::vector<std::vector<typename std::vector<VectorDIM>::size_type> > _convexhullpts;


    void addPoint(const VectorDIM& pt) {
      _pts.push_back(pt);
    }

    /*
     * Check if point is inside of the convex hull of the point cloud
     *
     * @test_needed for 2D_D, 3D_F, 3D_D
    */
    bool pointInside(const VectorDIM& pt) const {
      for(auto hp : _convexhull) {
        if(hp.signedDistance(pt) < 0) {
          return false;
        }
      }
      return true;
    }

    /*
     * Check if point is inside of the convex hull of the point cloud
     * where hyperplanes of the convex hull are shifted by shift so that
     * convex hull grows shift is positive
     *
     * @test_needed for 2D_D, 3D_F, 3D_D
     *
    */
    bool pointInside(const VectorDIM& pt, T shift) const {
      for(auto hp : _convexhull) {
        if(hp.signedDistance(pt) < -shift) {
          return false;
        }
      }
      return true;
    }

    /*
     * Compute the convex hull of the point cloud.
     *
     * points must be set beforehand, obviously.
     *
     * currently looking at every tuple of size DIM, which is highly inefficient.
     * can implement https://en.wikipedia.org/wiki/Quickhull
     *
     * @test_needed for 2D_D, 3D_F, 3D_D
     *
    */
    void convexHull() {
      assert(_pts.size() >= DIM);
      _convexhull.clear();
      _convexhullpts.clear();
      CombinationGenerator generator(_pts.size());
      for(auto comb = generator.begin(); !generator.end(); comb = generator.next()) {
        Hyperplane hp;
        try {
          hp = hyperplaneThroughPoints(comb);
        } catch(LinearDependenceException& exp) {
          continue;
        }

        bool everythingNegative = true;
        bool everythingPositive = true;
        for(typename std::vector<VectorDIM>::size_type i = 0; i < _pts.size(); i++) {
          if(std::find(comb.begin(), comb.end(), i) == comb.end()) {
            T dist = hp.signedDistance(_pts[i]);
            if(dist < 0) {
              everythingPositive = false;
            } else if(dist > 0) {
              everythingNegative = false;
            }
          }
        }

        /*
          Normals should point inside of the obstacle
        */
        if(everythingNegative) {
          hp.normal() = -1 * hp.normal();
          hp.offset() = -1 * hp.offset();
        } else if(everythingPositive) {

        } else {
          continue;
        }
        _convexhull.push_back(hp);
        _convexhullpts.push_back(comb);
      }

    }

    /*
     * Return true if convex hull intersects with the axis aligned box
     *
     * Look at p. 74 of Graphics Gems IV by Heckbert for better implementation
     *
     * Currently implemented specifically for 3D and 2D cases in a bad way!
    */
    virtual bool convexHullIntersects(const AlignedBox& box) const {
      return true;
    }


    /*
     * Getter for point
    */
    VectorDIM& operator[](typename std::vector<VectorDIM>::size_type i) {
      return _pts[i];
    }

    /*
     * Const getter for point
    */
    const VectorDIM& operator[](typename std::vector<VectorDIM>::size_type i) const {
      return _pts[i];
    }

    /*
     * Getter for convex hull hyperplane
    */
    Hyperplane& operator()(typename std::vector<Hyperplane>::size_type i) {
      return _convexhull[i];
    }

    /*
     * Const getter for convex hull hyperplane
    */
    const Hyperplane& operator()(typename std::vector<Hyperplane>::size_type i) const {
      return _convexhull[i];
    }

  private:

    /*
     * Find the hyperplane that goes through given points
    */
    Hyperplane hyperplaneThroughPoints(const std::vector<typename std::vector<VectorDIM>::size_type>& indexArray) {
      assert(indexArray.size() == DIM);
      MatrixDIM A;
      VectorDIM b;
      for(unsigned int i = 0; i < DIM; i++) {
        A.block(i, 0, 1, DIM) = _pts[indexArray[i]].transpose();
        b(i) = 1.0;
      }


      Eigen::FullPivLU<MatrixDIM> decomp(A);
      if(!decomp.isInvertible()) {
        throw LinearDependenceException("points are not linearly independent.");
      }

      VectorDIM normal = A.inverse() * b;
      T distance = -1.0 / normal.norm();
      normal.normalize();

      return Hyperplane(normal, distance);
    }


    /*
     * Generates combinations of size DIM
     * where values in each combination is less than upperbound
     * specifically for convex hull calculations
     *
     * used for each index combinations for PointCloud::_pts array.
    */
    class CombinationGenerator {


      public:
        /**
         * Element type of combinations
        */
        using ElementType = typename std::vector<VectorDIM>::size_type;


        /**
         * Size type for combination indexes and size.
        */
        using SizeType = typename std::vector<ElementType>::size_type;

        /*
         * constructor for combination generator where s is the size of _pts array.
         *
         * elements of the combinations will be in range [0, upperbound-1]
        */
        CombinationGenerator(const ElementType& s) : upperbound(s), comb(DIM, ElementType()), nomore(false) {

        }

        /*
         * Returns the first combination
        */
        const std::vector<ElementType>& begin() {
          if(DIM > upperbound) {
            nomore = true;
          } else {
            nomore = false;
          }
          for(SizeType i = 0; i < DIM; i++) {
            comb[i] = upperbound - i - 1;
          }
          return comb;
        }

        /*
         * Returns the next combination
         * Sets nomore variable to true if there isn't anymore combinations
        */
        const std::vector<ElementType>& next() {
          for(SizeType i = 0; i < DIM; i++) {
            T lb = i;
            SizeType cur = DIM - 1 - i;
            if(comb[cur] == lb) {

            } else {
              comb[cur]--;
              for(SizeType j = cur + 1; j < DIM; j++) {
                comb[j] = comb[j-1] - 1;
              }
              return comb;
            }
          }

          nomore = true;
          return comb;
        }

        /*
         * Returns true if there isn't anymore combinations.
        */
        bool end() {
          return nomore;
        }

      private:
        ElementType upperbound;
        std::vector<ElementType> comb;
        bool nomore;
    };

};

/*
 * Default point cloud for generic stuff
 *
*/
template<typename T, unsigned int DIM>
class PointCloud : public PointCloudBase<T, DIM> {

};


/*
 *
 * Point cloud template specialization for DIM = 2
 *
*/
template<typename T>
class PointCloud<T, 2U> : public PointCloudBase<T, 2U> {
  public:

    using typename PointCloudBase<T, 2U>::AlignedBox;
    using typename PointCloudBase<T, 2U>::VectorDIM;

    /*
     * Assumes that the point cloud is not completely inside the box!
     *
     * Does its computation completely in an inefficient manner that is samples
     * the faces of box with samplesPerDimension number of times per each
     * dimension
     *
     * samplesPerDimension must be at least 2 since there are two corners
     * per dimension.
     *
     * Look at p. 74 of Graphics Gems IV by Heckbert for better implementation
     * This is implemented just to save the day
     *
     * @test_needed for F and D
     *
    */
    bool convexHullIntersects(const AlignedBox& box) const override {
      unsigned int samplesPerDimension = 5;
      const T xmin = box.min()(0);
      const T ymin = box.min()(1);
      const T xmax = box.max()(0);
      const T ymax = box.max()(1);

      const T xstep = (xmax - xmin) / (samplesPerDimension - 1);
      const T ystep = (ymax - ymin) / (samplesPerDimension - 1);

      for(T x = xmin; x <= xmax; x += xstep) {
       for(T y = ymin; y <= ymax; y += ystep) {
         const VectorDIM pt(x, y);
         if(this->pointInside(pt)) {
           return true;
         }
       }
      }

      return false;
    }
};



/*
 *
 * Point cloud template specialization for DIM = 3
 *
*/
template<typename T>
class PointCloud<T, 3U> : public PointCloudBase<T, 3U> {
  public:

    using typename PointCloudBase<T, 3U>::AlignedBox;
    using typename PointCloudBase<T, 3U>::VectorDIM;
    /*
     * Assumes that the point cloud is not completely inside the box!
     *
     * Does its computation completely in an inefficient manner that is samples
     * the faces of box with samplesPerDimension number of times per each
     * dimension
     *
     * samplesPerDimension must be at least 2 since there are two corners
     * per dimension.
     *
     * Look at p. 74 of Graphics Gems IV by Heckbert for better implementation
     * This is implemented just to save the day
     *
     *
     * @test_needed for F and D
     *
    */
    bool convexHullIntersects(const AlignedBox& box) const override {
      unsigned int samplesPerDimension = 5;
      const T xmin = box.min()(0);
      const T ymin = box.min()(1);
      const T zmin = box.min()(2);
      const T xmax = box.max()(0);
      const T ymax = box.max()(1);
      const T zmax = box.max()(2);

      const T xstep = (xmax - xmin) / (samplesPerDimension - 1);
      const T ystep = (ymax - ymin) / (samplesPerDimension - 1);
      const T zstep = (zmax - zmin) / (samplesPerDimension - 1);

      for(T y = ymin; y <= ymax; y += ystep) {
        for(T z = zmin; z <= zmax; z += zstep) {
          const VectorDIM ptm(xmin, y, z);
          const VectorDIM ptM(xmax, y, z);
          if(this->pointInside(ptm) || this->pointInside(ptM)) {
            return true;
          }
        }
      }

      for(T x = xmin; x <= xmax; x += xstep) {
        for(T z = zmin; z <= zmax; z += zstep) {
          const VectorDIM ptm(x, ymin, z);
          const VectorDIM ptM(x, ymax, z);
          if(this->pointInside(ptm) || this->pointInside(ptM)) {
            return true;
          }
        }
      }

      for(T x = xmin; x <= xmax; x += xstep) {
        for(T y = ymin; y <= ymax; y += ystep) {
          const VectorDIM ptm(x, y, zmin);
          const VectorDIM ptM(x, y, zmax);
          if(this->pointInside(ptm) || this->pointInside(ptM)) {
            return true;
          }
        }
      }
      return false;
    }
};

}
#endif
