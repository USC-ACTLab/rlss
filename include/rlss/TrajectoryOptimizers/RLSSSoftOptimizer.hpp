#ifndef RLSS_RLSS_SOFT_OPTIMIZER_HPP
#define RLSS_RLSS_SOFT_OPTIMIZER_HPP

#include <rlss/TrajectoryOptimizers/TrajectoryOptimizer.hpp>
#include <rlss/internal/Util.hpp>

namespace rlss {

template<typename T, unsigned int DIM>
class RLSSSoftOptimizer: public TrajectoryOptimizer<T, DIM> {
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

    RLSSSoftOptimizer(
        std::shared_ptr<CollisionShape> colshape,
        const PiecewiseCurveQPGenerator& qpgen,
        const AlignedBox& ws,
        unsigned int contupto,
        const std::vector<std::pair<unsigned int, T>>& lambdas,
        const std::vector<T>& thetas,
        const std::unordered_map<std::string, std::pair<bool, T>>&
                    soft_parameters,
        T obstacle_check_distance
        ): m_collision_shape(colshape),
        m_qp_generator(qpgen),
        m_workspace(ws),
        m_continuity_upto(contupto),
        m_lambda_integrated_squared_derivatives(lambdas),
        m_theta_position_at(thetas),
        m_soft_parameters(soft_parameters),
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
                    mathematica,
                    m_soft_parameters
            );
        } catch(...) {
            return std::nullopt;
        }


        auto initial_guess = m_qp_generator.getDVarsForSegments(segments);
        auto problem = m_qp_generator.getProblem()
                            .convert_to_soft();
        debug_message("hard num vars: "
                    , m_qp_generator.getProblem().num_vars()
                    , ", soft num vars: "
                    , problem.num_vars());
        auto soft_initial_guess = Vector(problem.num_vars());
        soft_initial_guess.setZero();
        soft_initial_guess.block(0, 0, initial_guess.cols(), 1) = initial_guess;


        QPWrappers::RLSS_SOFT_QP_SOLVER::Engine<T> solver;
        solver.setFeasibilityTolerance(1e-9);
        Vector soln;
        QPWrappers::OptReturnType ret = QPWrappers::OptReturnType::Unknown;
        try {
            ret = solver.next(
                    problem, soln, soft_initial_guess);
        } catch(...) {
        }
        debug_message("optimization return value: ", ret);
        if(ret == QPWrappers::OptReturnType::Optimal) {
            debug_message("slack variables: ",
              soln.block(m_qp_generator.getProblem().num_vars(),
                         0,
                         soln.rows() - m_qp_generator.getProblem().num_vars(),
                         1).transpose()
            );
            Vector soln_primary
                = soln.block(0, 0, m_qp_generator.numDecisionVariables(), 1);
            debug_message("primary variables: ", soln_primary. transpose());
            auto result = m_qp_generator.extractCurve(soln_primary);
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
    std::unordered_map<std::string, std::pair<bool, T>> m_soft_parameters;
    T m_obstacle_check_distance;

}; // RLSSSoftOptimizer

} // namespace rlss

#endif // RLSS_RLSS_OPTIMIZER_HPP