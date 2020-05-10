#ifndef RLSS_DISCRETESEARCH_HPP
#define RLSS_DISCRETESEARCH_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <libMultiRobotPlanning/a_star.hpp>
#include <boost/functional/hash/hash.hpp>
#include <optional>


namespace rlss {

namespace internal {



template<typename T, unsigned int DIM>
std::optional<StdVectorVectorDIM<T, DIM>> discreteSearch(
            const typename OccupancyGrid<T, DIM>::Index& start_idx,
            const typename OccupancyGrid<T, DIM>::Index& goal_idx,
            const OccupancyGrid<T, DIM>& occupancy_grid,
            const AlignedBox<T, DIM>& workspace,
            std::shared_ptr<rlss::CollisionShape<T,DIM>> collision_shape
) {
    using VectorDIM = VectorDIM<T, DIM>;
    using OccupancyGrid = OccupancyGrid<T, DIM>;
    using AlignedBox = AlignedBox<T, DIM>;
    using StdVectorVectorDIM = StdVectorVectorDIM<T, DIM>;
    using Index = typename OccupancyGrid::Index;
    using Coordinate = typename OccupancyGrid::Coordinate;
    using CollisionShape = rlss::CollisionShape<T, DIM>;


    struct State {
        Index idx;
        Index dir;
        State(Index i) : idx(i), dir(Index::Zero()) {}
        State(Index i, Index d) : idx(i), dir(d) {}
        bool operator==(const State& rhs) const {
            return idx == rhs.idx && dir == rhs.dir;
        }
    };

    struct StateHasher {
        std::size_t operator()(const State& s) const {
            std::size_t seed = 0;
            for(unsigned int d = 0; d < DIM; d++) {
                boost::hash_combine(seed, s.idx(d));
                boost::hash_combine(seed, s.dir(d));
            }
            return seed;
        }
    };

    enum class Action {
        FORWARD,
        ROTATE
    };

    class Environment {
    public:
        Environment(
            const OccupancyGrid& occ, 
            const AlignedBox& works, 
            std::shared_ptr<CollisionShape> cols,
            const Index& goal
            )
            : m_occupancy_grid(occ),
              m_workspace(works),
              m_collision_shape(cols),
              m_goal(goal)
        {}

        int admissibleHeuristic(const State& s) {
            int h = 0;
            for(unsigned int d = 0; d < DIM; d++) {
                h += std::abs(s.idx(d) - m_goal(d));
            }
            return h;
        }

        bool isSolution(const State& s) { return s.idx == m_goal; }

        void getNeighbors(
            const State& s,
            std::vector<libMultiRobotPlanning::Neighbor<State, Action, int> >& 
                neighbors) {
            neighbors.clear();

            if(s.dir == Index::Zero()) {
                for(unsigned int d = 0; d < DIM; d++) {
                    Index dir = Index::Zero();
                    dir(d) = 1;
                    Index idx = s.idx + dir;
                    if(this->actionValid(s.idx, idx)) {
                        neighbors.emplace_back(State(idx, dir), Action::FORWARD, 1);
                    }

                    dir(d) = -1;
                    idx = s.idx + dir;
                    if(this->actionValid(s.idx, idx)) {
                        neighbors.emplace_back(State(idx, dir), Action::FORWARD, 1);
                    }
                }
            } else {
                Index idx = s.idx + s.dir;
                if(this->actionValid(s.idx, idx)) {
                    neighbors.emplace_back(State(idx, s.dir), Action::FORWARD, 1);
                }

                for(unsigned int d = 0; d < DIM; d++) {
                    if(s.dir(d) == 0) {
                        Index dir = Index::Zero();
                        dir(d) = 1;
                        neighbors.emplace_back(State(s.idx, dir), Action::ROTATE, 1);
                        dir(d) = -1;
                        neighbors.emplace_back(State(s.idx, dir), Action::ROTATE, 1);
                    }
                }
            }
        }

        void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

        void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

    public:
        bool actionValid(const Index& from_idx, const Index& to_idx) {
            Coordinate from_center = m_occupancy_grid.getCenter(from_idx);
            Coordinate to_center = m_occupancy_grid.getCenter(to_idx);

            AlignedBox from_box = m_collision_shape->boundingBox(from_center);
            AlignedBox to_box = m_collision_shape->boundingBox(to_center);

            from_box.extend(to_box);

            StdVectorVectorDIM from_pts = rlss::internal::cornerPoints<T, DIM>(from_box);

            for(const auto& pt : from_pts) {
                if(!m_workspace.contains(pt))
                    return false;
            }

            return !m_occupancy_grid.isOccupied(from_box);
        }

        bool indexValid(const Index& idx) {
            Coordinate center = m_occupancy_grid.getCenter(idx);
            AlignedBox robot_box = m_collision_shape->boundingBox(center);

            StdVectorVectorDIM robot_pts = rlss::internal::cornerPoints<T, DIM>(robot_box);

            for(const auto& pt : robot_pts) {
                if(!m_workspace.contains(pt))
                    return false;
            }

            return !m_occupancy_grid.isOccupied(robot_box);
        }

    private:
        const OccupancyGrid& m_occupancy_grid;
        const AlignedBox& m_workspace;
        std::shared_ptr<CollisionShape> m_collision_shape;
        const Index& m_goal;
    };

    Environment env(occupancy_grid, workspace, collision_shape, goal_idx);
    libMultiRobotPlanning::AStar<State, Action, int, Environment, StateHasher> astar(env);
    libMultiRobotPlanning::PlanResult<State, Action, int> solution;

    if(env.indexValid(start_idx)) {
        State start_state(start_idx);
        bool success = astar.search(start_state, solution);
        if(!success) {
            return std::nullopt;
        }

        StdVectorVectorDIM segments;
        segments.push_back(occupancy_grid.getCenter(solution.states[0].first.idx));
        for(std::size_t i = 0; i < solution.actions.size(); i++) {
            if(solution.actions[i].first == Action::ROTATE) {
                segments.push_back(occupancy_grid.getCenter(solution.states[i+1].first.idx));
            }
        }
        segments.push_back(occupancy_grid.getCenter(solution.states.back().first.idx));
        return segments;
    }

    return std::nullopt;
}

} // namespace internal
} // namespace rlss

#endif // RLSS_DISCRETESEARCH_HPP