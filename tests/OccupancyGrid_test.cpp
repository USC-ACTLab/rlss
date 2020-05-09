#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/OccupancyGrid.hpp>

TEST_CASE("OccupancyGrid.getIndex() test", "OccupancyGrid") {
    using OG = rlss::OccupancyGrid<double, 2>;
    using Index = OG::Index;
    using Coordinate = OG::Coordinate;
    using AlignedBox = OG::AlignedBox;

    OG grid(Coordinate(0.5, 0.4));
    Index idx = grid.getIndex(Coordinate(0.4, 0.3));
    REQUIRE(idx == Index(0, 0));

    idx = grid.getIndex(Coordinate(-0.6, 0.3));
    REQUIRE(idx == Index(-2, 0));

    idx = grid.getIndex(Coordinate(0.5, 0.4));
    REQUIRE(idx == Index(1, 1));

    idx = grid.getIndex(Coordinate(-0.5, -0.3));
    REQUIRE(idx == Index(-2, -1));

    idx = grid.getIndex(Coordinate(-0.5-1e-9, -0.3));
    REQUIRE(idx == Index(-2, -1));

    idx = grid.getIndex(Coordinate(-0.5+1e-9, -0.3));
    REQUIRE(idx == Index(-1, -1));

    idx = grid.getIndex(Coordinate(0, 0));
    REQUIRE(idx == Index(-1, -1));

    idx = grid.getIndex(Coordinate(1e-9, 1e-9));
    REQUIRE(idx == Index(0, 0));
}

TEST_CASE("OccupancyGrid.getCenter() test", "OccupancyGrid") {
    using OG = rlss::OccupancyGrid<double, 3>;
    using Index = OG::Index;
    using Coordinate = OG::Coordinate;
    using AlignedBox = OG::AlignedBox;

    OG grid(Coordinate(0.5, 0.4, 0.7));

    Coordinate center = grid.getCenter(Index(0, 0, 0));
    REQUIRE((center - Coordinate(0.25, 0.2, 0.35)).squaredNorm() < 1e-9);

    center = grid.getCenter(Index(-1, 1, 2));
    REQUIRE((center - Coordinate(-0.25, 0.6, 1.75)).squaredNorm() < 1e-9);

    center = grid.getCenter(Coordinate(122.3, 12.7, 11));
    REQUIRE((center - Coordinate(122.25, 12.6, 10.85)).squaredNorm() < 1e-9);
}