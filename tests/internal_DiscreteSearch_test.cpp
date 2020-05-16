#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include <rlss/internal/DiscreteSearch.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <optional>
#include <memory>

TEST_CASE("discrete search in 2d", "internal::DiscreteSearch") {
    using OccupancyGrid = rlss::OccupancyGrid<double, 2U>;
    using Coordinate = OccupancyGrid::Coordinate;
    using Index = OccupancyGrid::Index;
    using AlignedBox = OccupancyGrid::AlignedBox;
    using AlignedBoxCollisionShape = rlss::AlignedBoxCollisionShape<double, 2U>;
    using CollisionShape = rlss::CollisionShape<double, 2U>;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, 2U>;
    using VectorDIM = rlss::internal::VectorDIM<double, 2U>;

    OccupancyGrid grid(Coordinate(0.5, 0.5));
    grid.setOccupancy(Index(1,3));
    grid.setOccupancy(Index(2,4));
    grid.setOccupancy(Index(3,4));
    grid.setOccupancy(Index(4,5));
    grid.setOccupancy(Index(5,6));

    auto al_collision_shape = std::make_shared<AlignedBoxCollisionShape>(AlignedBox(VectorDIM(-0.35, -0.35), VectorDIM(0.35, 0.35)));
    auto collision_shape = std::static_pointer_cast<CollisionShape>(al_collision_shape);
    Coordinate start_position(0.74,0.75);
    Coordinate goal_position(3.76,3.75);

    AlignedBox workspace(VectorDIM(0, 0), VectorDIM(4.5, 4.5));

    auto result = rlss::internal::discreteSearch<double, 2U>(
                            start_position, 
                            goal_position, 
                            grid, 
                            workspace, 
                            collision_shape
    );

    REQUIRE(result != std::nullopt);

    StdVectorVectorDIM result_vector = *result;


    REQUIRE(result_vector.size() == 4);
    REQUIRE((result_vector[0] - VectorDIM(0.74, 0.75)).squaredNorm() < 1e-9);
    REQUIRE((result_vector[1] - VectorDIM(1.25, 0.75)).squaredNorm() < 1e-9);
    REQUIRE((result_vector[2] - VectorDIM(3.75, 0.75)).squaredNorm() < 1e-9);
    REQUIRE((result_vector[3] - VectorDIM(3.76, 3.75)).squaredNorm() < 1e-9);
}
