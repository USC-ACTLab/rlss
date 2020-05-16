#ifndef RLSS_DISCRETESEARCH_HPP
#define RLSS_DISCRETESEARCH_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <libMultiRobotPlanning/a_star.hpp>
#include <boost/functional/hash/hash.hpp>
#include <optional>
#include <iostream>


namespace rlss {

namespace internal {



template<typename T, unsigned int DIM>
std::optional<StdVectorVectorDIM<T, DIM>> discreteSearch(
            const typename OccupancyGrid<T, DIM>::Coordinate& start_coordinate,
            const typename OccupancyGrid<T, DIM>::Coordinate& goal_coordinate,
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
        Coordinate position;
        Index dir;
        explicit State(Coordinate c) : position(c), dir(Index::Zero()) {}
        State(Coordinate c, Index d) : position(c), dir(d) {}
        bool operator==(const State& rhs) const {
            return position == rhs.position && dir == rhs.dir;
        }
    };

    struct StateHasher {
        std::size_t operator()(const State& s) const {
            std::size_t seed = 0;
            for(unsigned int d = 0; d < DIM; d++) {
                boost::hash_combine(seed, s.position(d));
                boost::hash_combine(seed, s.dir(d));
            }
            return seed;
        }
    };

    enum class Action {
        FORWARD,
        ROTATE,
        ROTATEFORWARD
    };

    class Environment {
    public:
        Environment(
            const OccupancyGrid& occ, 
            const AlignedBox& works, 
            std::shared_ptr<CollisionShape> cols,
            const Coordinate& goal
            )
            : m_occupancy_grid(occ),
              m_workspace(works),
              m_collision_shape(cols),
              m_goal(goal)
        {}

        int admissibleHeuristic(const State& s) {
            int h = 0;
            for(unsigned int d = 0; d < DIM; d++) {
                h += std::abs(s.position(d) - m_goal(d));
            }
            return h;
        }

        bool isSolution(const State& s) { return s.position == m_goal; }

        void getNeighbors(
            const State& s,
            std::vector<libMultiRobotPlanning::Neighbor<State, Action, int> >& 
                neighbors) {

            neighbors.clear();

            Coordinate s_center = m_occupancy_grid.getCenter(s.position);
            Index s_idx = m_occupancy_grid.getIndex(s_center);

            if(this->actionValid(s.position, m_goal)) {
                neighbors.emplace_back(
                    State(m_goal, Index::Zero()), 
                    Action::ROTATEFORWARD, 
                    1 + (s_idx - m_occupancy_grid.getIndex(m_goal)).cwiseAbs().sum()
                );
            }

            if(s_center != s.position) { // get into grid
                if(this->actionValid(s.position, s_center)) {
                    neighbors.emplace_back(
                        State(s_center, Index::Zero()), 
                        Action::ROTATEFORWARD,
                        2
                    );
                }

                for(unsigned int i = 0; i < DIM; i++) {
                    Index new_idx = s_idx;
                    new_idx(i)++;
                    Coordinate new_idx_center = m_occupancy_grid.getCenter(new_idx);
                    if(this->actionValid(s.position, new_idx_center)) {
                        neighbors.emplace_back(
                            State(new_idx_center, Index::Zero()), 
                            Action::ROTATEFORWARD,
                            2
                        );
                    }

                    new_idx(i) -= 2;
                    new_idx_center = m_occupancy_grid.getCenter(new_idx);
                    if(this->actionValid(s.position, new_idx_center)) {
                        neighbors.emplace_back(
                            State(new_idx_center, Index::Zero()), 
                            Action::ROTATEFORWARD,
                            2
                        );
                    }
                }
            } else {
                if(s.dir == Index::Zero()) {
                    for(unsigned int d = 0; d < DIM; d++) {
                        Index dir = Index::Zero();
                        dir(d) = 1;
                        Index idx = s_idx + dir;
                        if(this->actionValid(s_idx, idx)) {
                            neighbors.emplace_back(
                                State(m_occupancy_grid.getCenter(idx), dir), 
                                Action::FORWARD, 
                                1
                            );
                        }

                        dir(d) = -1;
                        idx = s_idx + dir;
                        if(this->actionValid(s_idx, idx)) {
                            neighbors.emplace_back(
                                State(m_occupancy_grid.getCenter(idx), dir), 
                                Action::FORWARD, 
                                1
                            );
                        }
                    }
                } else {
                    Index idx = s_idx + s.dir;
                    if(this->actionValid(s_idx, idx)) {
                        neighbors.emplace_back(
                            State(m_occupancy_grid.getCenter(idx), s.dir), 
                            Action::FORWARD, 
                            1
                        );
                    }

                    for(unsigned int d = 0; d < DIM; d++) {
                        if(s.dir(d) == 0) {
                            Index dir = Index::Zero();
                            dir(d) = 1;
                            neighbors.emplace_back(
                                State(m_occupancy_grid.getCenter(s_idx), dir), 
                                Action::ROTATE, 
                                1
                            );
                            dir(d) = -1;
                            neighbors.emplace_back(
                                State(m_occupancy_grid.getCenter(s_idx), dir), 
                                Action::ROTATE, 
                                1
                            );
                        }
                    }
                }
            }

            
        }

        void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

        void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

    public:

        bool actionValid(const Coordinate& start, const Coordinate& end) {
            AlignedBox from_box = m_collision_shape->boundingBox(start);
            AlignedBox to_box = m_collision_shape->boundingBox(end);

            from_box.extend(to_box);

            StdVectorVectorDIM from_pts = rlss::internal::cornerPoints<T, DIM>(from_box);

            for(const auto& pt : from_pts) {
                if(!m_workspace.contains(pt))
                    return false;
            }

            return !m_occupancy_grid.isOccupied(from_box);
        }

        bool actionValid(const Index& from_idx, const Index& to_idx) {
            Coordinate from_center = m_occupancy_grid.getCenter(from_idx);
            Coordinate to_center = m_occupancy_grid.getCenter(to_idx);

            return this->actionValid(from_center, to_center);
        }


        bool positionValid(const Coordinate& pos) {
            AlignedBox robot_box = m_collision_shape->boundingBox(pos);

            StdVectorVectorDIM robot_pts = rlss::internal::cornerPoints<T, DIM>(robot_box);

            for(const auto& pt : robot_pts) {
                if(!m_workspace.contains(pt))
                    return false;
            }

            return !m_occupancy_grid.isOccupied(robot_box);
        }

        bool indexValid(const Index& idx) {
            Coordinate center = m_occupancy_grid.getCenter(idx);
            return this->positionValid(center);
        }

    private:
        const OccupancyGrid& m_occupancy_grid;
        const AlignedBox& m_workspace;
        std::shared_ptr<CollisionShape> m_collision_shape;
        const Coordinate& m_goal;
    };

    Environment env(occupancy_grid, workspace, collision_shape, goal_coordinate);
    libMultiRobotPlanning::AStar<State, Action, int, Environment, StateHasher> astar(env);
    libMultiRobotPlanning::PlanResult<State, Action, int> solution;

    if(env.positionValid(start_coordinate)) {
        State start_state(start_coordinate);
        bool success = astar.search(start_state, solution);
        if(!success) {
            return std::nullopt;
        }
        
        StdVectorVectorDIM segments;
        segments.push_back(solution.states[0].first.position);
        for(std::size_t i = 0; i < solution.actions.size(); i++) {
            if(solution.actions[i].first == Action::ROTATE) {
                segments.push_back(solution.states[i+1].first.position);
            } else if (solution.actions[i].first == Action::ROTATEFORWARD) {
                if(i != 0)
                    segments.push_back(solution.states[i].first.position);
                if(i != solution.actions.size() - 1)
                    segments.push_back(solution.states[i+1].first.position);
            }
        }
        segments.push_back(solution.states.back().first.position);

        return segments;
    }

    return std::nullopt;
}


} // namespace internal
} // namespace rlss

#endif // RLSS_DISCRETESEARCH_HPP