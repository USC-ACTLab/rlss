#ifndef RLSS_RLSS_HARD_OPTIMIZER_HPP
#define RLSS_RLSS_HARD_OPTIMIZER_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/TrajectoryOptimizers/TrajectoryOptimizer.hpp>
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <rlss/internal/SVM.hpp>
#include <rlss/internal/MathematicaWriter.hpp>
#include <chrono>
#include <rlss/internal/RLSSOptimization.hpp>

namespace rlss {

template<typename T, unsigned int DIM>
class RLSSHardOptimizer: public TrajectoryOptimizer<T, DIM> {
public:
    using Base = TrajectoryOptimizer<T, DIM>;
    using StdVectorVectorDIM = typename Base::StdVectorVectorDIM;
    using AlignedBox = typename Base::AlignedBox;
    using OccupancyGrid = typename Base::OccupancyGrid;
    using PiecewiseCurve = typename Base::PiecewiseCurve;
    using PiecewiseCurveQPGenerator = splx::PiecewiseCurveQPGenerator<T, DIM>;
    using VectorDIM = rlss::internal::VectorDIM<T, DIM>;
    using CollisionShape = rlss::CollisionShape<T, DIM>;
    using Hyperplane = rlss::internal::Hyperplane<T, DIM>;
    using Vector = typename PiecewiseCurveQPGenerator::Vector;

    RLSSHardOptimizer(
        std::shared_ptr<CollisionShape> colshape,
        const PiecewiseCurveQPGenerator& qpgen,
        const AlignedBox& ws,
        unsigned int contupto,
        const std::vector<std::pair<unsigned int, T>>& lambdas,
        const std::vector<T>& thetas,
        T obstacle_check_distance
    ): m_collision_shape(colshape),
       m_qp_generator(qpgen),
       m_workspace(ws),
       m_continuity_upto(contupto),
       m_lambda_integrated_squared_derivatives(lambdas),
       m_theta_position_at(thetas),
       m_obstacle_check_distance(obstacle_check_distance)
    {

    }


    // returns std::nullopt when optimization fails
    std::optional<PiecewiseCurve> optimize(
            const StdVectorVectorDIM& segments,
            const std::vector<T>& durations,
            const std::vector<AlignedBox>& oth_rbt_col_shape_bboxes,
            const OccupancyGrid& occupancy_grid,
            const StdVectorVectorDIM& current_robot_state
    )  override {

        internal::MathematicaWriter<T, DIM> mathematica;

        try {
            internal::generate_optimization_problem<T, DIM>(
                    m_qp_generator,
                    m_collision_shape,
                    m_workspace,
                    m_continuity_upto,
                    m_lambda_integrated_squared_derivatives,
                    m_theta_position_at,
                    m_obstacle_check_distance,
                    segments,
                    durations,
                    oth_rbt_col_shape_bboxes,
                    occupancy_grid,
                    current_robot_state,
                    mathematica
            );
        } catch(...) {
            return std::nullopt;
        }


        QPWrappers::RLSS_HARD_QP_SOLVER::Engine<T> solver;
        solver.setFeasibilityTolerance(1e-9);
        auto initial_guess = m_qp_generator.getDVarsForSegments(segments);
        Vector soln;
        QPWrappers::OptReturnType ret = QPWrappers::OptReturnType::Unknown;
        try {
            ret = solver.next(
                    m_qp_generator.getProblem(), soln, initial_guess);
        } catch (...) {
        }

        debug_message("optimization return value: ", ret);

        if(ret == QPWrappers::OptReturnType::Optimal) {
            auto result = m_qp_generator.extractCurve(soln);
            mathematica.piecewiseCurve(result);
            return result;
        } else {
            return std::nullopt;
        }
    }
private:
    std::shared_ptr<CollisionShape> m_collision_shape;
    PiecewiseCurveQPGenerator m_qp_generator;
    AlignedBox m_workspace;
    unsigned int m_continuity_upto;
    std::vector<std::pair<unsigned int, T>>
        m_lambda_integrated_squared_derivatives;
    std::vector<T> m_theta_position_at;
    T m_obstacle_check_distance;
}; // class TrajectoryOptimizer

}

#endif // RLSS_RLSS_HARD_OPTIMIZER_HPP