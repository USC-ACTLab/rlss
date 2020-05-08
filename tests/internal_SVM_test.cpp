#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/internal/SVM.hpp>
#include <iostream>

TEST_CASE("SVM in 2D", "internal::svm") {
    using StdVectorVectorDIM2 = rlss::internal::StdVectorVectorDIM<double, 2U>;
    using VectorDIM2 = rlss::internal::VectorDIM<double, 2U>;
    using Hyperplane2 = rlss::internal::Hyperplane<double, 2U>;

    StdVectorVectorDIM2 f {{2,1}, {2,7}, {1,5}, {3,1}};
    StdVectorVectorDIM2 s {{-1, -1}, {-1, -2}, {-2, -3}, {-3, -3}};

    Hyperplane2 svmhp = rlss::internal::svm<double, 2U>(f, s);

    for(const VectorDIM2& v: f) {
        REQUIRE(svmhp.normal().dot(v) + svmhp.offset() <= -1+1e-9);
    }
    for(const VectorDIM2& v: s) {
        REQUIRE(svmhp.normal().dot(v) + svmhp.offset() >= 1-1e-9);
    }

    Hyperplane2 shp {VectorDIM2 {-0.461538461538462, -0.307692307692308}, 0.230769230769231};

    REQUIRE(shp.isApprox(svmhp, 1e-9));
}


TEST_CASE("SVM in 3D", "internal::svm") {
    using StdVectorVectorDIM3 = rlss::internal::StdVectorVectorDIM<double, 3U>;
    using VectorDIM3 = rlss::internal::VectorDIM<double, 3U>;
    using Hyperplane3 = rlss::internal::Hyperplane<double, 3U>;

    StdVectorVectorDIM3 f {{-1,-1,-1}, {-1,-1,3}, {-1,2,-1}, {-1,2,3}, {1,-1,-1}, {1,-1,3}, {1,2,-1}, {1,2,3}};
    StdVectorVectorDIM3 s {{2,1,2}, {2,1,7}, {2,6,2}, {2,6,7}, {5,1,2}, {5,1,7}, {5,6,2}, {5,6,7}};

    Hyperplane3 svmhp = rlss::internal::svm<double, 3U>(f, s);

    for(const VectorDIM3& v: f) {
        REQUIRE(svmhp.normal().dot(v) + svmhp.offset() <= -1+1e-9);
    }
    for(const VectorDIM3& v: s) {
        REQUIRE(svmhp.normal().dot(v) + svmhp.offset() >= 1-1e-9);
    }

    Hyperplane3 shp {VectorDIM3 {2,0,0}, -3};

    REQUIRE(shp.isApprox(svmhp, 1e-9));
}