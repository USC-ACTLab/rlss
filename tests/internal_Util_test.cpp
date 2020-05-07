#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <rlss/internal/Util.hpp>
#include <iostream>

TEST_CASE("Corner points are computed for 3D", "[internal::cornerPoints]") {
    using AlignedBox3 = rlss::internal::AlignedBox<double, 3U>;
    using VectorDIM3 = rlss::internal::VectorDIM<double, 3U>;
    using StdVectorVectorDIM3 = rlss::internal::StdVectorVectorDIM<double, 3U>;

    AlignedBox3 box(VectorDIM3(-1, 1, 2), VectorDIM3(3, 2, 1));
    StdVectorVectorDIM3 corners = rlss::internal::cornerPoints<double, 3U>(box);

    REQUIRE(corners.size() == 8);
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(-1, 1, 2)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(-1, 1, 1)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(-1, 2, 2)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(-1, 2, 1)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(3, 1, 2)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(3, 1, 1)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(3, 2, 2)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM3(3, 2, 1)) != corners.end());
}

TEST_CASE("Corner points are computed for 2D", "[internal::cornerPoints]") {
    using AlignedBox2 = rlss::internal::AlignedBox<double, 2U>;
    using VectorDIM2 = rlss::internal::VectorDIM<double, 2U>;
    using StdVectorVectorDIM2 = rlss::internal::StdVectorVectorDIM<double, 2U>;

    AlignedBox2 box(VectorDIM2(-1, 1), VectorDIM2(3, 2));
    StdVectorVectorDIM2 corners = rlss::internal::cornerPoints<double, 2U>(box);

    REQUIRE(corners.size() == 4);
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM2(-1, 1)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM2(-1, 2)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM2(3, 1)) != corners.end());
    REQUIRE(std::find(corners.begin(), corners.end(), VectorDIM2(3, 2)) != corners.end());
}