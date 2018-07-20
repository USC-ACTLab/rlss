#ifndef PATHREPLAN_OBSTACLE_H
#define PATHREPLAN_OBSTACLE_H
#include <vector>
#include "vectoreuc.h"
#include "hyperplane.h"
#include <Eigen/Dense>
#include <iostream>

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
    void ch_planes(double shift);
    bool point_inside(vectoreuc& pt);
    bool point_inside(vectoreuc& pt, double shift);
    double closest_distance(vectoreuc& pt); // should be called only when point is inside
    double biggest_negative_distance(vectoreuc& pt); // should be called only when point is outside
};

/**
 * T will be float, double or long double presumably.
 *
 * DIM is the dimension points are in.
*/
template <typename T, unsigned int DIM>
class PointCloud {

  public:
    typedef Eigen::Matrix<T, DIM, 1> Vector;
    typedef Eigen::Hyperplane<T, DIM> Hyperplane;

    /*
     * Point cloud
    */
    std::vector<Vector> _pts;

    /*
     * Check if point is inside of the convex hull of the point cloud
    */
    bool pointInside(const Vector& pt) const {
      return true;
    }

    /*
     * Check if point is inside of the convex hull of the point cloud
     * where hyperplanes of the convex hull are shifted by shift so that
     * convex hull grows
    */
    bool pointInside(const Vector& pt, double shift) const {
      return true;
    }

    /*
     * Compute the convex hull of the point cloud.
     *
     * points must be set beforehand, obviously.
     *
     * currently looking at every tuple of size DIM, which is highly inefficient.
     * can implement https://en.wikipedia.org/wiki/Quickhull
    */
    void convexHull() {
      CombinationGenerator generator(_pts.size());
      int i = 0;
      for(auto comb = generator.begin(); !generator.end(); comb = generator.next(), i++) {
        
      }

    }

    /*
     * Getter for point
    */
    Vector& operator[](typename std::vector<Vector>::size_type i) const {
      return _pts[i];
    }

    /*
     * Getter for convex hull hyperplane
    */
    Hyperplane& operator()(typename std::vector<Hyperplane>::size_type i) const {
      return _convexhull[i];
    }

  private:
    /*
     * Hyperplanes of the convex hull where normals are pointing inside the convex hull.
    */
    std::vector<Hyperplane> _convexhull;

    /*
     * Generates combinations of size DIM
     * where values in each combination is less than upperbound
     * specifically for convex hull calculations
     *
     * used for each index combinations for PointCloud::_pts array.
    */
    class CombinationGenerator {

      /**
       * Element type of combinations
      */
      typedef typename std::vector<Vector>::size_type ElementType;


      /**
       * Size type for combination indexes and size.
      */
      typedef typename std::vector<ElementType>::size_type SizeType;

      public:
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

#endif
