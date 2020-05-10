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

namespace rlss {

/*
*   T: field type used by the algorithm, e.g. double
*/
template<typename T, unsigned int DIM>
class RLSS {
public:
    using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
    using PiecewiseCurveQPGenerator = splx::PiecewiseCurveQPGenerator<T, DIM>; 
    using _OccupancyGrid = rlss::OccupancyGrid<T, DIM>;
    using AlignedBox = Eigen::AlignedBox<T, DIM>;
    using VectorDIM = Eigen::Matrix<T, DIM, 1>;
    using Hyperplane = Eigen::Hyperplane<T, DIM>;
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    using StdVectorVectorDIM 
        = std::vector<VectorDIM, Eigen::aligned_allocator<VectorDIM>>;

    _OccupancyGrid& getOccupancyGrid() {
        return m_occupancy_grid;
    }

    const _OccupancyGrid& getOccupancyGrid() const {
        return m_occupancy_grid;
    }

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
                    other_robot_collision_shape_bounding_boxes) {

        std::vector<Hyperplane> robot_safety_hyperplanes 
                = robotSafetyHyperplanes(
                        current_robot_state[0],
                        other_robot_collision_shape_bounding_boxes);

        for(const auto& bbox: other_robot_collision_shape_bounding_boxes) {
            m_occupancy_grid.addTemporaryObstacle(bbox);
        }


        T target_time_in_original_trajectory;
        bool target_time_valid = false;

        if(current_time + m_planning_horizon > m_original_trajectory.maxParameter()) {
            target_time_in_original_trajectory = m_original_trajectory.maxParameter();
            target_time_valid 
                = m_occupancy_grid.isOccupied(
                            m_collision_shape->boundingBox(
                                        m_original_trajectory.eval(
                                            target_time_in_original_trajectory, 
                                            0
                                        )
                            )
            );

            while(!target_time_valid && target_time_in_original_trajectory >= 0) {
                target_time_in_original_trajectory -= m_search_step;
                target_time_valid 
                    = m_occupancy_grid.isOccupied(
                                m_collision_shape->boundingBox(
                                            m_original_trajectory.eval(
                                                target_time_in_original_trajectory, 
                                                0
                                            )
                                )
                );
            }
        } else {
            target_time_in_original_trajectory = current_time + m_planning_horizon;
            target_time_valid 
                = m_occupancy_grid.isOccupied(
                            m_collision_shape->boundingBox(
                                        m_original_trajectory.eval(
                                            target_time_in_original_trajectory, 
                                            0
                                        )
                            )
            );
            
            bool forward_search = true;
            bool forward_doable = true;
            bool backward_doable = true;
            unsigned int step = 1;
            while(!target_time_valid && (forward_doable || backward_doable)) {
                if(forward_search && forward_doable) {
                    target_time_in_original_trajectory += step*m_search_step;
                    forward_search = false;
                    if(target_time_in_original_trajectory > m_original_trajectory.maxParameter()) {
                        forward_doable = false;
                        continue;
                    }
                } else if(!forward_search && backward_doable) {
                    target_time_in_original_trajectory -= step * m_search_step;
                    forward_search = true;
                    step++;
                    if(target_time_in_original_trajectory < 0) {
                        backward_doable = false;
                        continue;
                    }
                }

                if((!forward_search && forward_doable) || (forward_search && backward_doable)) {
                    target_time_valid
                        = m_occupancy_grid.isOccupied(
                                    m_collision_shape->boundingBox(
                                                m_original_trajectory.eval(
                                                    target_time_in_original_trajectory, 
                                                    0
                                                )
                                    )
                    );
                }
            }
        }
    
        if(!target_time_valid) {
            return std::nullopt;
        }

        StdVectorVectorDIM path = discreteSearch(
                                    m_occupancy_grid.getIndex(current_robot_state[0]), 
                                    m_occupancy_grid.getIndex(
                                        m_original_trajectory.eval(
                                            target_time_in_original_trajectory, 
                                            0
                                        )
                                    ),
                                    m_occupancy_grid,
                                    m_workspace,
                                    m_collision_shape
        );


        m_occupancy_grid.clearTemporaryObstacles();
    }

private:

    // collision shape of the robot
    std::shared_ptr<CollisionShape> m_collision_shape; 

    // occupancy grid as seen by the robot
    _OccupancyGrid m_occupancy_grid;

    // original trjaectory of the robot
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

    // maximum velocity of obstacles in the environment
    T m_max_obstacle_velocity;

    // parameter search step on curves for collision checking operations.
    T m_search_step;

    // plan should be continous up to this degree
    unsigned int m_continuity_upto;
    
    // each pair [d, lambda] denotes the lambda for integrated squared d^th
    // derivative of the curve
    std::vector<std::pair<unsigned int, T>> m_lambda_integrated_squared_derivatives;

    // each pair [d, l] denotes that magnitude of d^th derivative should
    // not be more than l
    std::vector<std::pair<unsigned int, T>> m_max_derivative_magnitudes;

    // thetas for position costs for each of the curve piece endpoints.
    // must satisfy m_theta_positions_at.size() == m_qp_generator.numPieces()
    std::vector<T> m_theta_position_at;


    // shift hyperplane hp creating hyperplane shp
    // such that whenever the center_of_mass of the robot
    // is to the negative side of the shp, 
    // the ch_points (convex hull points) of collision shape of the
    // robot is to the negative side of the hyperplane hp.
    Hyperplane shiftHyperplane(const VectorDIM& center_of_mass, 
                               const StdVectorVectorDIM& ch_points, 
                               const Hyperplane& hp) const {
        Hyperplane shp {hp.normal(), hp.offset()};

        T offset = std::numeric_limits<T>::max();
        for(const auto& pt : ch_points) {
            shp.offset() 
                = std::min(offset, hp.normal().dot(center_of_mass - pt));
        }

        return shp;
    }

    std::vector<Hyperplane> robotSafetyHyperplanes(
        const VectorDIM& robot_position,
        const std::vector<AlignedBox>& 
                other_robot_collision_shape_bounding_boxes) const {

        std::vector<Hyperplane> hyperplanes;
        
        StdVectorVectorDIM robot_points 
            = m_collision_shape->boundingBox(robot_position);

        for(const auto& oth_collision_shape_bbox: other_robot_collision_shape_bounding_boxes) {
            StdVectorVectorDIM oth_points = rlss::internal::cornerPoints(oth_collision_shape_bbox);
            Hyperplane svm_hp = rlss::internal::svm(robot_points, oth_collision_shape);

            Hyperplane svm_shifted = this->shiftHyperplane(
                                        robot_position, 
                                        robot_points, 
                                        svm_hp
            );
            hyperplanes.push_back(svm_shifted);
        }

        return hyperplanes;
    }

}; // class RLSS


} // namespace rlss
#endif // RLSS_RLSS_HPP