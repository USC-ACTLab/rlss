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
        State(Index i) : idx(i), dir(Index::Zeros()) {}
        State(Index i, Index d) : idx(i), dir(d) {}
    };

    struct StateHasher {
        std::size_t operator()(const State& s) {
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
            const AlignedBox* works, 
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

            if(s.dir == VectorDIM::Zero()) {
                for(unsigned int d = 0; d < DIM; d++) {
                    Index dir = Index::Zeros();
                    dir(d) = 1;
                    Index idx = s.idx + dir;
                    if(this->indexValid(idx)) {
                        neighbors.emplace_back(State(idx, dir), Action::FORWARD, 1);
                    }

                    dir(d) = -1;
                    idx = s.idx + dir;
                    if(this->stateValid(idx)) {
                        neighbors.emplace_back(State(idx, dir), Action::FORWARD, 1);
                    }
                }
            } else {
                Index idx = s.idx + s.dir;
                if(this->indexValid(idx)) {
                    neighbors.emplace_back(State(idx, s.dir), Action::FORWARD, 1);
                }

                for(unsigned int d = 0; d < DIM; d++) {
                    if(s.dir(d) == 0) {
                        Index dir = Index::Zeros();
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
        bool indexValid(const Index& idx) {
            Coordinate center = m_occupancy_grid.getCenter(idx);
            StdVectorVectorDIM robot_pts = m_collision_shape->convexHullPoints(center);
            for(const auto& pt : robot_pts) {
                if(!m_workspace.contains(pt))
                    return false;
            }

            return !m_occupancy_grid.isOccupied(robot_pts);
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
        segments.push_back(occupancy_grid.getCenter(solution.back().first.idx));
        return segments;
    }

    return std::nullopt;
}

} // namespace internal
} // namespace rlss

#endif // RLSS_DISCRETESEARCH_HPP