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
    using _OccupancyGrid = OccupancyGrid<T, DIM>;
    using Coordinate = typename _OccupancyGrid::Coordinate;
    using Index = typename _OccupancyGrid::Index;
    using UnorderedIndexSet = typename OccupancyGrid<T, DIM>::UnorderedIndexSet;

    Index start_idx = occupancy_grid.getIndex(start_position);
    AlignedBox start_box = collision_shape->boundingBox(start_position);

    UnorderedIndexSet reachable;
    UnorderedIndexSet visited;
    std::queue<Index> q;


    AlignedBox to_box = collision_shape->boundingBox(
        occupancy_grid.getCenter(start_idx)
    );
    to_box.extend(start_box);

    if(!occupancy_grid.isOccupied(to_box) && workspace.contains(to_box)) {
        q.push(start_idx);
        visited.insert(start_idx);
    }

    Index idx = Index::Zero();
    for(unsigned int i = 0; i < DIM; i++) {
        idx(i) = 1;
        Index nw_idx = start_idx + idx;
        AlignedBox to_box = collision_shape->boundingBox(
                                occupancy_grid.getCenter(nw_idx)
        );

        to_box.extend(start_box);

        if(!occupancy_grid.isOccupied(to_box) && workspace.contains(to_box)) {
            q.push(nw_idx);
            visited.insert(nw_idx);
        }

        idx(i) = -1;
        nw_idx = start_idx + idx;
        to_box = collision_shape->boundingBox(
                                occupancy_grid.getCenter(nw_idx)
        );

        to_box.extend(start_box);

        if(!occupancy_grid.isOccupied(to_box) && workspace.contains(to_box)) {
            q.push(nw_idx);
            visited.insert(nw_idx);
        }
    }

    while(!q.empty()) {
        Index& fr = q.front();
        AlignedBox from_box = collision_shape->boundingBox(
            occupancy_grid.getCenter(fr)
        );

        reachable.insert(fr);

        Index dir = Index::Zero();
        for(unsigned int j = 0; j < DIM; j++) {
            for(int i = -1; i < 2; i+=2) {
                dir(j) = i;
                Index new_idx = fr + dir;
                if(visited.find(new_idx) == visited.end()) {
                    AlignedBox to_box = collision_shape->boundingBox(
                        occupancy_grid.getCenter(new_idx)
                    );
                    to_box.extend(from_box);
                    if(!occupancy_grid.isOccupied(to_box) && workspace.contains(to_box)) {
                        q.push(new_idx);
                        visited.insert(new_idx);
                    }
                }
            }
            dir(j) = 0; 
        }

        q.pop();
    }


    return reachable;
}

} // namespace internal
} // namespace rlss

#endif // RLSS_INTERNAL_BFS_HPP