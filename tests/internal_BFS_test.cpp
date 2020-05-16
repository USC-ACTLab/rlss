#include "rlss/CollisionShapes/CollisionShape.hpp"
#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/internal/BFS.hpp>
#include <memory>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/internal/Util.hpp>
#include <unordered_set>

TEST_CASE("BFS in 2D", "[bfs]") {
    using _OccupancyGrid = rlss::OccupancyGrid<double, 2U>;
    using AlColShape = rlss::AlignedBoxCollisionShape<double, 2U>;
    using ColShape = rlss::CollisionShape<double, 2U>;
    using AlignedBox = typename _OccupancyGrid::AlignedBox;
    using VectorDIM = typename _OccupancyGrid::VectorDIM;
    using Coordinate = _OccupancyGrid::Coordinate;
    using Index = _OccupancyGrid::Index;
    using UnorderedIndexSet = _OccupancyGrid::UnorderedIndexSet;

    auto collision_shape_al 
        = std::make_shared<AlColShape>(
            AlignedBox(VectorDIM{-0.33, -0.33}, VectorDIM{0.33, 0.33})
    );
    auto collision_shape = std::static_pointer_cast<ColShape>(collision_shape_al);

    _OccupancyGrid grid(Coordinate(0.5, 0.5));

    grid.setOccupancy(Index(5, 5));
    grid.setOccupancy(Index(5, 8));
    grid.setOccupancy(Index(11, 12));

    AlignedBox workspace(VectorDIM(0,0), VectorDIM(7, 7));

    UnorderedIndexSet bfs_expected_result;

    for(long long int i = 0; i < 14; i++) {
        for(long long int j = 0; j < 14; j++) {
            bfs_expected_result.emplace(i, j);
        }
    }

    for(long long int i = 0; i < 14; i++) {
        bfs_expected_result.erase(Index(0, i));
        bfs_expected_result.erase(Index(i, 0));
        bfs_expected_result.erase(Index(i, 13));
        bfs_expected_result.erase(Index(13, i));
    }

    for(long long int i = 4; i < 7; i++) {
        for(long long int j = 4; j < 10; j++) {
            bfs_expected_result.erase(Index(i, j));
        }
    }

    for(long long int i = 10; i < 13; i++) {
        for(long long int j = 11; j < 13; j++) {
            bfs_expected_result.erase(Index(i, j));
        }
    }

    UnorderedIndexSet bfs_result = rlss::internal::BFS<double, 2U>(
                            VectorDIM(1.2, 5), 
                            grid, 
                            workspace, 
                            collision_shape
    );

    REQUIRE(bfs_result == bfs_expected_result);

    grid.setOccupancy(Index(7, 9));
    grid.setOccupancy(Index(8, 10));
    grid.setOccupancy(Index(5, 2));

    for(long long int i = 4; i < 14; i++) {
        for(long long int j = 0; j < 10; j++) {
            bfs_expected_result.erase(Index(i, j));
        }
    }

    for(long long int i = 6; i < 14; i++)
        bfs_expected_result.erase(Index(i, 10));
    for(long long int i = 7; i < 14; i++)
        bfs_expected_result.erase(Index(i, 11));

    bfs_result = rlss::internal::BFS<double, 2U>(
                            VectorDIM(1.2, 5), 
                            grid, 
                            workspace, 
                            collision_shape
    );


    REQUIRE(bfs_result == bfs_expected_result);
}