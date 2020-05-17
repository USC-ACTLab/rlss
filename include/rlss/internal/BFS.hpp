#ifndef RLSS_INTERNAL_BFS_HPP
#define RLSS_INTERNAL_BFS_HPP

#include <rlss/OccupancyGrid.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <memory>
#include <iostream>

namespace rlss {
namespace internal {

// returns all occupancy grid indexes reachable from the start_position without
// a collision using discrete segments
template<typename T, unsigned int DIM>
typename OccupancyGrid<T, DIM>::UnorderedIndexSet BFS(
    const VectorDIM<T, DIM>& start_position,
    const OccupancyGrid<T, DIM>& occupancy_grid,
    const AlignedBox<T, DIM>& workspace,
    std::shared_ptr<CollisionShape<T, DIM>> collision_shape
) {
    using VectorDIM = VectorDIM<T, DIM>;
    using AlignedBox = AlignedBox<T, DIM>;
    using OccupancyGrid = OccupancyGrid<T, DIM>;
    using Coordinate = typename OccupancyGrid::Coordinate;
    using Index = typename OccupancyGrid::Index;
    using UnorderedIndexSet = typename OccupancyGrid::UnorderedIndexSet;

    Index start_idx = occupancy_grid.getIndex(start_position);
    AlignedBox start_box = collision_shape->boundingBox(start_position);

    UnorderedIndexSet reachable;
    std::queue<Index> q;

    if(rlss::internal::segmentValid<T, DIM>(
            occupancy_grid,
            workspace,
            start_box,
            start_idx,
            collision_shape))
    {
        q.push(start_idx);
        reachable.insert(start_idx);
    }

    std::vector<Index> start_neighbors = occupancy_grid.getNeighbors(start_idx);
    for(const auto& neigh_idx : start_neighbors) {
        if(rlss::internal::segmentValid<T, DIM>(
                occupancy_grid,
                workspace,
                start_box,
                neigh_idx,
                collision_shape))
        {
            q.push(neigh_idx);
            reachable.insert(neigh_idx);
        }
    }

    while(!q.empty()) {
        Index& fr = q.front();
        std::vector<Index> neighbors = occupancy_grid.getNeighbors(fr);
        AlignedBox fr_box = collision_shape->boundingBox(
                occupancy_grid.getCenter(fr)
        );
        for(const auto& neigh_idx : neighbors) {
            if(reachable.find(neigh_idx) == reachable.end()
                && rlss::internal::segmentValid<T, DIM>(
                    occupancy_grid,
                    workspace,
                    fr_box,
                    neigh_idx,
                    collision_shape))
            {
                q.push(neigh_idx);
                reachable.insert(neigh_idx);
            }
        }
        q.pop();
    }


    return reachable;
}

} // namespace internal
} // namespace rlss

#endif // RLSS_INTERNAL_BFS_HPP