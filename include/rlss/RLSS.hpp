#ifndef RLSS_RLSS_HPP
#define RLSS_RLSS_HPP

#include <rlss/OccupancyGrid.hpp>
#include <rlss/internal/SVM.hpp>
#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/cplex.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <limits>
#include <optional>
#include <rlss/internal/BFS.hpp>
#include <rlss/internal/DiscreteSearch.hpp>
#include <cassert>

namespace rlss {

/*
*   T: field type used by the algorithm, e.g. double
*/
template<typename T, unsigned int DIM>
class RLSS {
public:
    using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
    using PiecewiseCurveQPGenerator = splx::PiecewiseCurveQPGenerator<T, DIM>; 
    using OccupancyGrid = rlss::OccupancyGrid<T, DIM>;
    using Index = typename OccupancyGrid::Index;
    using AlignedBox = Eigen::AlignedBox<T, DIM>;
    using VectorDIM = Eigen::Matrix<T, DIM, 1>;
    using Hyperplane = Eigen::Hyperplane<T, DIM>;
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    using CollisionShape = rlss::CollisionShape<T, DIM>;

    using StdVectorVectorDIM 
        = std::vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>>;

    using UnorderedIndexSet = typename OccupancyGrid::UnorderedIndexSet;

    PiecewiseCurve& getOriginalTrajectory() {
        return m_original_trajectory;
    }

    const PiecewiseCurve& getOriginalTrajectory() const {
        return m_original_trajectory;
    }


    // plan for the current_time.
    std::optional<PiecewiseCurve> plan(
        T current_time, 
        const StdVectorVectorDIM& current_robot_state,
        const std::vector<AlignedBox>& 
                    other_robot_collision_shape_bounding_boxes,
        OccupancyGrid& occupancy_grid) {

        if(current_time < 0) {
            throw std::domain_error(
                absl::StrCat
                (
                    "current time can't be negative. given: ",
                    current_time
                )
            );
        }

        if(current_robot_state.size() < m_continuity_upto + 1) {
            throw std::domain_error
            (
                absl::StrCat
                (
                    "can't enforce contiuity up to degree",
                    m_continuity_upto,
                    " using robot state with number of elements ",
                    current_robot_state.size()
                )
            );
        }

        for(const auto& colbox: other_robot_collision_shape_bounding_boxes) {
            occupancy_grid.addTemporaryObstacle(colbox);
        }

        std::optional<std::pair<VectorDIM, T>> goal_and_duration 
                = this->goalSelection(current_robot_state[0],
                                      occupancy_grid,
                                      current_time
        );

        if(!goal_and_duration) {
            occupancy_grid.clearTemporaryObstacles();
            return std::nullopt;
        }


        std::optional<std::pair<StdVectorVectorDIM, std::vector<T>>>
             segments_and_durations =
                    this->discreteSearch(current_robot_state[0],
                                         goal_and_duration->first,
                                         goal_and_duration->second,
                                         occupancy_grid
        );

        if(!segments_and_durations) {
            occupancy_grid.clearTemporaryObstacles();
            return std::nullopt;
        }

        const StdVectorVectorDIM& segments = segments_and_durations->first;
        std::vector<T>& durations = segments_and_durations->second;


        std::optional<PiecewiseCurve> resulting_curve = std::nullopt;
        for(unsigned int c = 0; c < m_maximum_rescaling_count; c++) {
            resulting_curve = 
                    this->trajectoryOptimization(
                        segments, 
                        durations, 
                        other_robot_collision_shape_bounding_boxes,
                        occupancy_grid, 
                        current_robot_state
            );

            if( resulting_curve == std::nullopt 
                || this->needsTemporalRescaling(*resulting_curve)) 
            {
                for(auto& dur : durations) {
                    dur *= m_rescaling_duration_multipler;
                }
            } else {
                break; // curve is valid
            }
        }

        occupancy_grid.clearTemporaryObstacles();

        return resulting_curve;

    }

private:

    // collision shape of the robot
    std::shared_ptr<CollisionShape> m_collision_shape; 

    // original trajectory of the robot
    PiecewiseCurve m_original_trajectory;

    // qp generator for the piecewise curve. number of pieces and their
    // number of control points are implicitly provided by this structure.
    // durations of the pieces can be adjusted by the algorithm.
    PiecewiseCurveQPGenerator m_qp_generator;

    // robots must stay in this workspace
    AlignedBox m_workspace;

    // plan should have this duration
    T m_planning_horizon;

    // plan should be safe upto this time
    T m_safe_upto;

    // parameter search step on curves for collision checking operations.
    T m_search_step;

    // plan should be continuous up to this degree
    unsigned int m_continuity_upto;
    
    // duration multiplier when maximum derivative magnitude check fails.
    T m_rescaling_duration_multipler;

    // number of times to rescale the resulting curve when maximum derivative
    // magnitudes are more than required
    unsigned int m_maximum_rescaling_count;

    // each pair [d, lambda] denotes the lambda for integrated squared d^th
    // derivative of the curve
    std::vector<std::pair<unsigned int, T>>
        m_lambda_integrated_squared_derivatives;

    // each pair [d, l] denotes that magnitude of d^th derivative should
    // not be more than l
    std::vector<std::pair<unsigned int, T>> m_max_derivative_magnitudes;

    // thetas for position costs for each of the curve piece endpoints.
    // must satisfy m_theta_positions_at.size() == m_qp_generator.numPieces()
    std::vector<T> m_theta_position_at;


    std::optional<std::pair<VectorDIM, T>> goalSelection(
            const VectorDIM& current_position,
            const OccupancyGrid& occupancy_grid,
            T current_time) const 
    {
        UnorderedIndexSet reachable_indices 
                = rlss::internal::BFS<T, DIM>(
                        current_position, 
                        occupancy_grid, 
                        m_workspace,
                        m_collision_shape
        );

        T target_time = std::min(
                current_time + m_planning_horizon,
                m_original_trajectory.maxParameter()
        );
        VectorDIM target_position = m_original_trajectory.eval(target_time, 0);
        bool target_time_valid = false;
        std::vector<Index> neighbors = occupancy_grid.getNeighbors(
            target_position
        );
        neighbors.push_back(occupancy_grid.getIndex(target_position));
        for(const Index& neigh_idx: neighbors) {
            if(
                reachable_indices.find(neigh_idx) != reachable_indices.end()
                && rlss::internal::segmentValid<T, DIM>(
                    occupancy_grid,
                    m_workspace,
                    target_position,
                    neigh_idx,
                    m_collision_shape
                  )
            ) {
                target_time_valid = true;
                break;
            }
        }

        bool forward = true;
        long long int step_count = 1;
        while(
                !target_time_valid
                && (target_time + step_count * m_search_step
                        <= m_original_trajectory.maxParameter()
                    || target_time - step_count * m_search_step >= 0)
        ) {
            T candidate_target_time = target_time;
            if(forward) {
                candidate_target_time += step_count * m_search_step;
            } else {
                candidate_target_time -= step_count * m_search_step;
            }

            if(
                candidate_target_time >= 0
                && candidate_target_time <= m_original_trajectory.maxParameter()
            ) {
                VectorDIM candidate_target_position
                    = m_original_trajectory.eval(candidate_target_time, 0);
                std::vector<Index> neighbors = occupancy_grid.getNeighbors(
                        candidate_target_position
                );
                neighbors.push_back(
                    occupancy_grid.getIndex(candidate_target_position)
                );

                for(const Index& neigh_idx: neighbors) {
                    if(
                        reachable_indices.find(neigh_idx)
                            != reachable_indices.end()
                        && rlss::internal::segmentValid<T, DIM>(
                            occupancy_grid,
                            m_workspace,
                            candidate_target_position,
                            neigh_idx,
                            m_collision_shape
                        )
                    ) {
                        target_time = candidate_target_time;
                        target_position = candidate_target_position;
                        target_time_valid = true;
                        break;
                    }
                }
            }

            if(forward) {
                forward = false;
            } else {
                forward = true;
                step_count++;
            }
        }

        if(!target_time_valid) {
            return std::nullopt;
        }

        T actual_horizon = target_time - current_time;
        return make_pair(target_position, actual_horizon);
    }

    std::optional<std::pair<StdVectorVectorDIM, std::vector<T>>>
    discreteSearch(
        const VectorDIM& current_position,
        const VectorDIM& goal_position,
        T time_horizon,
        const OccupancyGrid& occupancy_grid
    ) const {
        time_horizon = std::max(time_horizon, m_safe_upto);

        std::optional<StdVectorVectorDIM> discrete_path_opt =
                rlss::internal::discreteSearch(
                    current_position,
                    goal_position,
                    occupancy_grid,
                    m_workspace,
                    m_collision_shape
        );

        if(!discrete_path_opt) {
            return std::nullopt;
        }

        StdVectorVectorDIM discrete_path = std::move(*discrete_path_opt);

        for(const auto& [d, l]: m_max_derivative_magnitudes) {
            if(d == 1) {
                T total_path_length = 0;
                for(std::size_t i = 0; i < discrete_path.size() - 1; i++) {
                    total_path_length
                        += (discrete_path[i+1] - discrete_path[i]).norm();
                }
                time_horizon = std::max(time_horizon, total_path_length/l);
            }
        }

        if(discrete_path.size() > m_qp_generator.numPieces() + 1) {
            discrete_path.resize(m_qp_generator.numPieces() + 1);
        } else if(discrete_path.size()  < m_qp_generator.numPieces() + 1) {
            discrete_path = rlss::internal::bestSplitSegments(
                    discrete_path,
                    m_qp_generator.numPieces()
            );
        }

        std::vector<T> segment_lengths(m_qp_generator.numPieces());
        T total_path_length = 0;
        for(std::size_t i = 0; i + 1 < discrete_path.size(); i++) {
            segment_lengths[i]
                    = (discrete_path[i+1] - discrete_path[i]).norm();
            total_path_length
                    += segment_lengths[i];
        }


        std::vector<T> segment_durations(m_qp_generator.numPieces(), 0);
        for(std::size_t i = 0; i < segment_lengths.size(); i++) {
            segment_durations[i]
                    = time_horizon * (segment_lengths[i] / total_path_length);
        }

        if(!segment_durations.empty() && segment_durations[0] < m_safe_upto) {
            T multiplier = m_safe_upto / segment_durations[0];
            segment_durations[0] = m_safe_upto;
            for(std::size_t i = 1; i < segment_durations.size(); i++) {
                segment_durations[i] *= multiplier;
            }
        }

        return {discrete_path, segment_durations};
    }

    std::optional<PiecewiseCurve> trajectoryOptimization(
            const StdVectorVectorDIM& segments,
            const std::vector<T>& durations,
            const std::vector<AlignedBox>& oth_rbt_col_shape_bboxes,
            const OccupancyGrid& occupancy_grid,
            const StdVectorVectorDIM& current_robot_state
    ) const {
        assert(segments.size() == m_qp_generator.numPieces() + 1);
        assert(durations.size() == segments.size());
        assert(current_robot_state.size() > m_continuity_upto);

        m_qp_generator.resetProblem();
        m_qp_generator.setPieceMaxParameters(durations);

        // robot to robot avoidance constraints for the first piece
        std::vector<Hyperplane> robot_to_robot_hps
                = this->robotSafetyHyperplanes(
                        current_robot_state[0],
                        oth_rbt_col_shape_bboxes
        );
        for(const auto& hp: robot_to_robot_hps) {
            m_qp_generator.addHyperplaneConstraintForPiece(0, hp);
        }

        // robot to obstacle avoidance constraints for all pieces
        for (
            std::size_t p_idx = 0;
            p_idx < m_qp_generator.numPieces();
            p_idx++
        ) {
            AlignedBox from_box 
                = m_collision_shape->boundingBox(segments[p_idx]);
            AlignedBox to_box 
                = m_collision_shape->boundingBox(segments[p_idx+1]);
            to_box.extend(from_box);

            StdVectorVectorDIM segments_corners 
                = rlss::internal::cornerPoints<T, DIM>(to_box);

            for(
                auto it = occupancy_grid.begin(); 
                it != occupancy_grid.end(); 
                it++
            ) {
                AlignedBox grid_box = *it;
                StdVectorVectorDIM grid_box_corners 
                    = rlss::internal::cornerPoints<T, DIM>(grid_box);
                Hyperplane shp = rlss::internal::svm
                (
                    segments_corners,
                    grid_box_corners
                );
                shp = rlss::internal::shiftHyperplane(
                    VectorDIM::Zero(), 
                    m_collision_shape->boundingBox(VectorDIM::Zero()), 
                    shp
                );

                m_qp_generator.addHyperplaneConstraintForPiece(p_idx, shp);
            }
        }

        // continuity constraints
        for(
            std::size_t p_idx = 0; 
            p_idx < m_qp_generator.numPieces() - 1; 
            p_idx++
        ) {
            for(unsigned int k = 0; k <= m_continuity_upto; k++) {
                m_qp_generator.addContinuityConstraint(p_idx, k);
            }
        }

        // initial point constraints
        for(unsigned int k = 0; k <= m_continuity_upto; k++) {
            m_qp_generator.addEvalConstraint(0, k, current_robot_state[k]);
        }

        // energy cost
        for(const auto& [d, l]: m_lambda_integrated_squared_derivatives) {
            m_qp_generator.addIntegratedSquaredDerivativeCost(d, l);
        }

        // eval cost 
        T duration_sum_before = 0;
        for(
            std::size_t p_idx = 0; 
            p_idx < m_qp_generator.numPieces(); 
            duration_sum_before += durations[p_idx], p_idx++
        ) {
            m_qp_generator.addEvalCost(
                duration_sum_before + durations[p_idx], 
                segments[p_idx+1], 
                m_theta_position_at[p_idx]
            );
        }

        QPWrappers::CPLEX::Engine<T> cplex;
        cplex.setFeasibilityTolerance(1e-9);

        auto initial_guess = m_qp_generator.getDVarsForSegments(segments);
        Vector soln;
        auto ret = cplex.next(
                        m_qp_generator.getProblem(), soln,  initial_guess);

        if(ret == QPWrappers::OptReturnType::Optimal) {
            return m_qp_generator.extractCurve(soln);
        }
        return std::nullopt;
    }

    std::vector<Hyperplane> robotSafetyHyperplanes(
        const VectorDIM& robot_position,
        const std::vector<AlignedBox>& 
                other_robot_collision_shape_bounding_boxes) const {

        std::vector<Hyperplane> hyperplanes;
        
        AlignedBox robot_box
            = m_collision_shape->boundingBox(robot_position);

        StdVectorVectorDIM robot_points
                = rlss::internal::cornerPoints<T, DIM>(robot_box);

        for(const auto& oth_collision_shape_bbox:
                    other_robot_collision_shape_bounding_boxes) {
            StdVectorVectorDIM oth_points
                    = rlss::internal::cornerPoints(oth_collision_shape_bbox);
            Hyperplane svm_hp = rlss::internal::svm(robot_points, oth_points);

            Hyperplane svm_shifted = rlss::internal::shiftHyperplane(
                                        robot_position, 
                                        robot_box,
                                        svm_hp
            );

            hyperplanes.push_back(svm_shifted);
        }

        return hyperplanes;
    }

}; // class RLSS


} // namespace rlss
#endif // RLSS_RLSS_HPP