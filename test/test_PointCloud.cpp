#include "PointCloud.h"
#include <algorithm>
#include <vector>
#include "gtest/gtest.h"
#include <cassert>

class PointCloudFloat2D : public ::testing::Test {
protected:
  ACT::PointCloud<float, 2> pc;

  ACT::PointCloud<float, 2> pc2;

  void SetUp() override {
    using namespace ACT;
    PointCloud<float, 2>::Vector vec;
    vec(0) = 3; vec(1) = -1; // 0
    pc._pts.push_back(vec);
    vec(0) = 4.5; vec(1) = 2; // 1
    pc._pts.push_back(vec);
    vec(0) = 1.2; vec(1) = 3.3; // 2
    pc._pts.push_back(vec);
    vec(0) = -1.0; vec(1) = 3.323; // 3
    pc._pts.push_back(vec);
    vec(0) = 12; vec(1) = 1.234; // 4
    pc._pts.push_back(vec);
    vec(0) = 7; vec(1) = -2; // 5
    pc._pts.push_back(vec);
    vec(0) = -12.12; vec(1) = 2.13212; // 6
    pc._pts.push_back(vec);
    vec(0) = -6.23; vec(1) = -1.231; // 7
    pc._pts.push_back(vec);
    vec(0) = 12.7; vec(1) = 5.22; // 8
    pc._pts.push_back(vec);
    vec(0) = 23.221231; vec(1) = 0.00002; // 9
    pc._pts.push_back(vec);
    vec(0) = 11.2231; vec(1) = 17.2223; // 10
    pc._pts.push_back(vec);
    pc.convexHull();


    vec(0) = -1.2; vec(1) = 0.7; pc2._pts.push_back(vec); // 0
    vec(0) = -1.2; vec(1) = -0.7; pc2._pts.push_back(vec); // 1
    vec(0) = 0.998; vec(1) = 0.7; pc2._pts.push_back(vec); // 2
    vec(0) = 0.91211; vec(1) = 0.41231; pc2._pts.push_back(vec);// 3
    vec(0) = 0.512; vec(1) = 0.233; pc2._pts.push_back(vec); // 4
    vec(0) = 0.122; vec(1) = 0.2411; pc2._pts.push_back(vec);// 5
    vec(0) = 0.998; vec(1) = -0.7; pc2._pts.push_back(vec);// 6
    vec(0) = 0; vec(1) = -0.22; pc2._pts.push_back(vec); // 7
    pc2.convexHull();
  }

  void TearDown() override {

  }

  bool ptOut(const ACT::PointCloud<float, 2>::Vector& vec) const {
    float x = vec(0);
    float y = vec(1);
    if(x < -11.5)
        return true;
    if(x > 23.5)
        return true;
    if (y > 17.5)
        return true;
    if(y < -2.8)
        return true;
    if( x > -11.5 && x < -10.5 && y < 2.5 && y > 1.5)
        return false;
    if( x > -11.5 && x < -10.5)
        return true;


    if( x > -10.5 && x < -9.5 && y < 3.5 && y > 0.5)
        return false;
    if( x > -10.5 && x < -9.5)
        return true;



    if( x > -9.5 && x < -8.5 && y < 4.5 && y > 0.5)
        return false;
    if( x > -9.5 && x < -8.5)
        return true;

    if( x > -8.5 && x < -7.5 && y < 4.5 && y > -0.5)
        return false;
    if( x > -8.5 && x < -7.5)
        return true;


    if( x > -7.5 && x < -6.5 && y < 5.5 && y > -0.5)
        return false;
    if( x > -7.5 && x < -6.5)
        return true;



    if( x > -6.5 && x < -5.5 && y < 6.5 && y > -1.5)
        return false;
    if( x > -6.5 && x < -5.5)
        return true;


    if( x > -5.5 && x < -4.5 && y < 6.5 && y > -1.5)
        return false;
    if( x > -5.5 && x < -4.5)
        return true;


    if( x > -4.5 && x < -3.5 && y < 7.5 && y > -1.5)
        return false;
    if( x > -4.5 && x < -3.5)
        return true;



    if( x > -3.5 && x < -2.5 && y < 8.5 && y > -1.5)
        return false;
    if( x > -3.5 && x < -2.5)
        return true;


    if( x > -2.5 && x < -1.5 && y < 8.5 && y > -1.5)
        return false;
    if( x > -2.5 && x < -1.5)
        return true;



    if( x > -1.5 && x < -0.5 && y < 9.5 && y > -1.5)
        return false;
    if( x > -1.5 && x < -0.5)
        return true;


    if( x > -0.5 && x < 0.5 && y < 9.5 && y > -1.5)
        return false;
    if( x > -0.5 && x < 0.5)
        return true;


    if( x > 0.5 && x < 1.5 && y < 10.5 && y > -1.5)
        return false;
    if( x > 0.5 && x < 1.5)
        return true;


    if( x > 1.5 && x < 2.5 && y < 11.5 && y > -1.5)
        return false;
    if( x > 1.5 && x < 2.5)
        return true;

    if( x > 2.5 && x < 3.5 && y < 11.5 && y > -1.5)
        return false;
    if( x > 2.5 && x < 3.5)
        return true;


    if( x > 3.5 && x < 4.5 && y < 12.5 && y > -1.5)
        return false;
    if( x > 3.5 && x < 4.5)
        return true;


    if( x > 4.5 && x < 5.5 && y < 13.5 && y > -1.5)
        return false;
    if( x > 4.5 && x < 5.5)
        return true;


    if( x > 5.5 && x < 6.5 && y < 13.5 && y > -1.5)
        return false;
    if( x > 5.5 && x < 6.5)
        return true;


    if( x > 6.5 && x < 7.5 && y < 14.5 && y > -2.5)
        return false;
    if( x > 6.5 && x < 7.5)
        return true;


    if( x > 7.5 && x < 8.5 && y < 15.5 && y > -1.5)
        return false;
    if( x > 7.5 && x < 8.5)
        return true;



    if( x > 8.5 && x < 9.5 && y < 15.5 && y > -1.5)
        return false;
    if( x > 8.5 && x < 9.5)
        return true;


    if( x > 9.5 && x < 10.5 && y < 16.5 && y > -1.5)
        return false;
    if( x > 9.5 && x < 10.5)
        return true;


    if( x > 10.5 && x < 11.5 && y < 17.5 && y > -1.5)
        return false;
    if( x > 10.5 && x < 11.5)
        return true;



    if( x > 11.5 && x < 12.5 && y < 16.5 && y > -1.5)
        return false;
    if( x > 11.5 && x < 12.5)
        return true;


    if( x > 12.5 && x < 13.5 && y < 14.5 && y > -1.5)
        return false;
    if( x > 12.5 && x < 13.5)
        return true;

    if( x > 13.5 && x < 14.5 && y < 13.5 && y > -1.5)
        return false;
    if( x > 13.5 && x < 14.5)
        return true;


    if( x > 14.5 && x < 15.5 && y < 11.5 && y > -1.5)
        return false;
    if( x > 14.5 && x < 15.5)
        return true;

    if( x > 15.5 && x < 16.5 && y < 10.5 && y > -0.5)
        return false;
    if( x > 15.5 && x < 16.5)
        return true;


    if( x > 16.5 && x < 17.5 && y < 8.5 && y > -0.5)
        return false;
    if( x > 16.5 && x < 17.5)
        return true;


    if( x > 17.5 && x < 18.5 && y < 7.5 && y > -0.5)
        return false;
    if( x > 17.5 && x < 18.5)
        return true;



    if( x > 18.5 && x < 19.5 && y < 6.5 && y > -0.5)
        return false;
    if( x > 18.5 && x < 19.5)
        return true;

    if( x > 19.5 && x < 20.5 && y < 4.5 && y > -0.5)
        return false;
    if( x > 19.5 && x < 20.5)
        return true;
    if( x > 20.5 && x < 21.5 && y < 3.5 && y > -0.5)
        return false;
    if( x > 20.5 && x < 21.5)
        return true;

    if( x > 21.5 && x < 22.5 && y < 1.5 && y > -0.5)
        return false;
    if( x > 21.5 && x < 22.5)
        return true;
    if( x > 22.5 && x < 23.5 && y < 0.5 && y > -0.5)
        return false;
    if( x > 22.5 && x < 23.5)
        return true;

    assert(1==2);
  }

  bool ptOutShifted(const ACT::PointCloud<float, 2>::Vector& vec) const {
    float x = vec(0), y = vec(1);
    if (y > 1.5) {
      return true;
    }

    if (y < -1.5)
     return true;

     if(x > 1.5)
      return true;

   return false;
  }
};


TEST_F(PointCloudFloat2D, ConvexHullComputedHyperplanesTest) {
  using namespace ACT;

  ASSERT_TRUE(pc._convexhull.size() == 5) << "pc._convexhull should have 5 hyperplanes but instead it has " << pc._convexhull.size();
  ASSERT_TRUE(pc._convexhullpts.size() == 5) << "pc._convexhullpts should have 5 hyperplanes but instead it has " << pc._convexhullpts.size();
  for(unsigned int i = 0; i < 5; i++) {
    ASSERT_TRUE(pc._convexhullpts[i].size() == 2) << "pc._convexhullpts[" << i <<"] should have 2 points but instead it has " << pc._convexhullpts[i].size();;
  }

  std::vector<std::vector<std::vector<PointCloud<float, 2>::Vector>::size_type> > chpts;

  std::vector<std::vector<PointCloud<float, 2>::Vector>::size_type> pt;
  pt.push_back(9); pt.push_back(10); chpts.push_back(pt); pt.clear();
  pt.push_back(10); pt.push_back(6); chpts.push_back(pt); pt.clear();
  pt.push_back(6); pt.push_back(7); chpts.push_back(pt); pt.clear();
  pt.push_back(5); pt.push_back(9); chpts.push_back(pt); pt.clear();
  pt.push_back(7); pt.push_back(5); chpts.push_back(pt); pt.clear();

  ASSERT_TRUE(std::is_permutation(chpts.begin(), chpts.end(), pc._convexhullpts.begin(),
              [] (const auto& f, const auto& s) -> bool {
                    return std::is_permutation(f.begin(), f.end(), s.begin());
                  })) << "pc._convexhullpts is not correct.";


  ASSERT_TRUE(pc2._convexhull.size() == 4) << "pc2._convexhull should have 4 hyperplanes but instead it has " << pc._convexhull.size();
  ASSERT_TRUE(pc2._convexhullpts.size() == 4) << "pc2._convexhullpts should have 4 hyperplanes but instead it has " << pc._convexhullpts.size();
  for(unsigned int i = 0; i < 4; i++) {
    ASSERT_TRUE(pc2._convexhullpts[i].size() == 2) << "pc2._convexhullpts[" << i <<"] should have 2 points but instead it has " << pc._convexhullpts[i].size();;
  }

  chpts.clear();
  pt.push_back(1); pt.push_back(6); chpts.push_back(pt); pt.clear();
  pt.push_back(6); pt.push_back(2); chpts.push_back(pt); pt.clear();
  pt.push_back(2); pt.push_back(0); chpts.push_back(pt); pt.clear();
  pt.push_back(0); pt.push_back(1); chpts.push_back(pt); pt.clear();

  ASSERT_TRUE(std::is_permutation(chpts.begin(), chpts.end(), pc2._convexhullpts.begin(),
              [] (const auto& f, const auto& s) -> bool {
                    return std::is_permutation(f.begin(), f.end(), s.begin());
                  })) << "pc2._convexhullpts is not correct.";

}

TEST_F(PointCloudFloat2D, ConvexHullPointsInsideTest) {
  using namespace ACT;

  PointCloud<float, 2>::Vector vec;
  for(float x = -25.0; x <= 25.0; x += 1.0) {
    for(float y = -25.0; y <= 25.0; y += 1.0) {
      vec(0) = x; vec(1) = y;
      if(ptOut(vec)) {
        EXPECT_FALSE(pc.pointInside(vec)) << "point " <<  x << ", " << y << " is outside of convex hull, but pointInside returns true";
      } else {
        EXPECT_TRUE(pc.pointInside(vec)) << "point " <<  x << ", " << y << " is inside of convex hull, but pointInside returns false";
      }
    }
  }
}

TEST_F(PointCloudFloat2D, ConvexHullPointsInsideShiftedTest) {
  using namespace ACT;


  for(unsigned int i = 0; i < pc2._pts.size(); i++) {
    ASSERT_TRUE(pc2.pointInside(pc2._pts[i], 0.5)) << "pc2._pts[" << i << "] is inside of shifted convex hull, but pointInside returns false";
  }


  ASSERT_FALSE(pc2.pointInside(pc2._pts[1], -0.000001)) << "pc2._pts[" << 1 << "] is outside of shifted convex hull, but pointInside returns true";
  ASSERT_FALSE(pc2.pointInside(pc2._pts[6], -0.000001)) << "pc2._pts[" << 6 << "] is outside of shifted convex hull, but pointInside returns true";
  ASSERT_FALSE(pc2.pointInside(pc2._pts[2], -0.000001)) << "pc2._pts[" << 2 << "] is outside of shifted convex hull, but pointInside returns true";
  ASSERT_FALSE(pc2.pointInside(pc2._pts[0], -0.000001)) << "pc2._pts[" << 0 << "] is outside of shifted convex hull, but pointInside returns true";

  ASSERT_TRUE(pc2.pointInside(pc2._pts[1], 0.000001)) << "pc2._pts[" << 1 << "] is inside of shifted convex hull, but pointInside returns false";
  ASSERT_TRUE(pc2.pointInside(pc2._pts[6], 0.000001)) << "pc2._pts[" << 6 << "] is inside of shifted convex hull, but pointInside returns false";
  ASSERT_TRUE(pc2.pointInside(pc2._pts[2], 0.000001)) << "pc2._pts[" << 2 << "] is inside of shifted convex hull, but pointInside returns false";
  ASSERT_TRUE(pc2.pointInside(pc2._pts[0], 0.000001)) << "pc2._pts[" << 0 << "] is inside of shifted convex hull, but pointInside returns false";



  PointCloud<float, 2>::Vector vec;
  for(float x = -2.0; x <= 2.0; x += 1.0) {
    for(float y = -2.0; y <= 2.0; y += 1.0) {
      vec(0) = x; vec(1) = y;
      if(ptOutShifted(vec)) {
        EXPECT_FALSE(pc2.pointInside(vec, 1.0)) << "point " <<  x << ", " << y << " is outside of the shifted convex hull, but pointInside returns true";
      } else {
        EXPECT_TRUE(pc2.pointInside(vec, 1.0)) << "point " <<  x << ", " << y << " is inside of the shifted convex hull, but pointInside returns false";
      }
    }
  }
}
