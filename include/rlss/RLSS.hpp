#ifndef RLSS_RLSS_HPP
#define RLSS_RLSS_HPP

#include <rlss/GoalSelectors/RLSSGoalSelector.hpp>
#include <rlss/TrajectoryOptimizers/RLSSOptimizer.hpp>
#include <rlss/DiscretePathSearchers/RLSSDiscretePathSearcher.hpp>
#include <rlss/ValidityCheckers/ValidityChecker.hpp>
#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <rlss/internal/Statistics.hpp>
#include <chrono>

namespace rlss {
template<typename T, unsigned int DIM>
class RLSS {
public:
    using GoalSelector_ = GoalSelector<T, DIM>;
    using TrajectoryOptimizer_ = TrajectoryOptimizer<T, DIM>;
    using DiscretePathSearcher_ = DiscretePathSearcher<T, DIM>;
    using ValidityChecker_ = ValidityChecker<T, DIM>;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<T, DIM>;
    using AlignedBox = rlss::internal::AlignedBox<T, DIM>;
    using VectorDIM = rlss::internal::VectorDIM<T, DIM>;
    using OccupancyGrid = rlss::OccupancyGrid<T, DIM>;
    using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;
    using StatisticsStorage = internal::StatisticsStorage<T>;
    using DurationStatistics = internal::DurationStatistics<T>;
    using SuccessFailureStatistics = internal::SuccessFailureStatistics<T>;

    RLSS(
        std::shared_ptr<GoalSelector_> goal_selector,
        std::shared_ptr<TrajectoryOptimizer_> trajectory_optimizer,
        std::shared_ptr<DiscretePathSearcher_> discrete_path_searcher,
        std::shared_ptr<ValidityChecker_> validity_checker,
        unsigned int max_rsca_count,
        T rsca_mul
    ) : m_goal_selector(goal_selector),
        m_trajectory_optimizer(trajectory_optimizer),
        m_discrete_path_searcher(discrete_path_searcher),
        m_validity_checker(validity_checker),
        m_maximum_rescaling_count(max_rsca_count),
        m_rescaling_duration_multipler(rsca_mul)
    {

    }

    std::optional<PiecewiseCurve> plan(
            T current_time,
            const StdVectorVectorDIM& current_robot_state,
            const std::vector<AlignedBox>&
            other_robot_collision_shape_bounding_boxes,
            OccupancyGrid& occupancy_grid) {

        DurationStatistics duration_statistics;
        SuccessFailureStatistics sf_statistics;

        auto plan_start_time = std::chrono::steady_clock::now();

        debug_message("planning...");

        for(std::size_t i = 0; i < current_robot_state.size(); i++) {
            debug_message(
                    "current robot state at degree ",
                    i,
                    " is ",
                    current_robot_state[i].transpose()
            );
        }

        if(current_time < 0) {
            throw std::domain_error(
                absl::StrCat
                    (
                        "current time can't be negative. given: ",
                        current_time
                    )
            );
        }


        for(const auto& colbox: other_robot_collision_shape_bounding_boxes) {
            occupancy_grid.addTemporaryObstacle(colbox);
        }

        debug_message("current position is ",
                      current_robot_state[0].transpose());
        debug_message("current time is ", current_time);

        debug_message("goalSelection...");



        auto goal_selector_start_time = std::chrono::steady_clock::now();
        std::optional<std::pair<VectorDIM, T>> goal_and_duration
                = m_goal_selector->select(current_robot_state[0],
                                      occupancy_grid,
                                      current_time
                );
        auto goal_selector_end_time = std::chrono::steady_clock::now();
        duration_statistics.setGoalSelectionDuration(
            std::chrono::duration_cast<std::chrono::microseconds>(
                goal_selector_end_time - goal_selector_start_time
            ).count()
        );

        if(!goal_and_duration) {
            debug_message(
                    internal::debug::colors::RED,
                    "goalSelection failed.",
                    internal::debug::colors::RESET
            );

            sf_statistics.setGoalSelectionSuccessFail(false);
            occupancy_grid.clearTemporaryObstacles();
            return std::nullopt;
        } else {
            sf_statistics.setGoalSelectionSuccessFail(true);
            debug_message(
                    internal::debug::colors::GREEN,
                    "goalSelection success.",
                    internal::debug::colors::RESET
            );
        }


        debug_message("goal position: ", goal_and_duration->first.transpose());
        debug_message("actual time horizon: ", goal_and_duration->second);


        debug_message("discreteSearch...");

        auto discrete_search_start_time = std::chrono::steady_clock::now();
        std::optional<std::pair<StdVectorVectorDIM, std::vector<T>>>
                segments_and_durations =
                m_discrete_path_searcher->search(
                    current_robot_state[0],
                    goal_and_duration->first,
                    goal_and_duration->second,
                    occupancy_grid
        );
        auto discrete_search_end_time = std::chrono::steady_clock::now();
        duration_statistics.setDiscreteSearchDuration(
                std::chrono::duration_cast<std::chrono::microseconds>(
                        discrete_search_end_time - discrete_search_start_time
                ).count()
        );

        if(!segments_and_durations) {
            debug_message(
                    internal::debug::colors::RED,
                    "discreteSearch failed.",
                    internal::debug::colors::RESET
            );
            occupancy_grid.clearTemporaryObstacles();
            sf_statistics.setDiscreteSearchSuccessFail(false);
            return std::nullopt;
        } else {
            debug_message(
                    internal::debug::colors::GREEN,
                    "discreteSearch success.",
                    internal::debug::colors::RESET
            );
            sf_statistics.setDiscreteSearchSuccessFail(true);
        }


        const StdVectorVectorDIM& segments = segments_and_durations->first;
        std::vector<T>& durations = segments_and_durations->second;

        debug_message(
                "segments.size() = ",
                segments.size(),
                ", durations.size() = ",
                durations.size()
        );
        assert(segments.size() == durations.size() + 1);

        for(std::size_t i = 0; i < durations.size(); i++) {
            debug_message(
                    "segment ",
                    i,
                    " is from ",
                    segments[i].transpose(),
                    " to ",
                    segments[i+1].transpose(),
                    " with duration ",
                    durations[i]
            );
        }

        std::optional<PiecewiseCurve> resulting_curve = std::nullopt;

        for(unsigned int c = 0; c < m_maximum_rescaling_count; c++) {
            debug_message("trajectoryOptimization...");
            auto trajectory_optimization_start_time
                = std::chrono::steady_clock::now();
            resulting_curve =
                    m_trajectory_optimizer->optimize(
                            segments,
                            durations,
                            other_robot_collision_shape_bounding_boxes,
                            occupancy_grid,
                            current_robot_state
                    );
            auto trajectory_optimization_end_time
                = std::chrono::steady_clock::now();

            duration_statistics.addTrajectoryOptimizationDuration(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                            discrete_search_end_time - discrete_search_start_time
                    ).count()
            );

            if(resulting_curve == std::nullopt) {
                debug_message(
                        internal::debug::colors::RED,
                        "trajectoryOptimization failed.",
                        internal::debug::colors::RESET
                );
                sf_statistics.addTrajectoryOptimizationSuccessFail(false);
            } else {
                debug_message(
                        internal::debug::colors::GREEN,
                        "trajectoryOptimization success.",
                        internal::debug::colors::RESET
                );
                sf_statistics.addTrajectoryOptimizationSuccessFail(true);
            }

            bool is_valid;
            if(resulting_curve != std::nullopt) {
                auto validity_checker_start_time = std::chrono::steady_clock::now();
                is_valid = m_validity_checker->isValid(*resulting_curve);
                auto validity_checker_end_time = std::chrono::steady_clock::now();
                duration_statistics.addTrajectoryOptimizationDuration(
                        std::chrono::duration_cast<std::chrono::microseconds>(
                                validity_checker_end_time -
                                validity_checker_start_time
                        ).count()
                );
            }

            if( resulting_curve == std::nullopt
                || !is_valid)
            {
                debug_message("doing temporal rescaling...");
                for(auto& dur : durations) {
                    dur *= m_rescaling_duration_multipler;
                }
            } else {
                debug_message(
                        internal::debug::colors::GREEN,
                        "does not need temporal rescaling.",
                        internal::debug::colors::RESET
                );
                break; // curve is valid
            }
        }

        occupancy_grid.clearTemporaryObstacles();

        auto plan_end_time = std::chrono::steady_clock::now();
        duration_statistics.setPlanningDuration(
            std::chrono::duration_cast<std::chrono::microseconds>(
                    plan_end_time - plan_start_time
            ).count()
        );

        debug_message("re-planning done.");
        if(resulting_curve == std::nullopt
           || !m_validity_checker->isValid(*resulting_curve)) {
            debug_message(
                    internal::debug::colors::RED,
                    "result: fail",
                    internal::debug::colors::RESET
            );
            sf_statistics.setPlanningSuccessFail(false);
            statistics_storage.add(sf_statistics);
            statistics_storage.add(duration_statistics);
            return std::nullopt;
        }
        else {
            debug_message(
                    internal::debug::colors::GREEN,
                    "result: success",
                    internal::debug::colors::RESET
            );
            sf_statistics.setPlanningSuccessFail(true);
            statistics_storage.add(sf_statistics);
            statistics_storage.add(duration_statistics);
            return resulting_curve;
        }
    }

private:
    std::shared_ptr<GoalSelector_> m_goal_selector;
    std::shared_ptr<TrajectoryOptimizer_> m_trajectory_optimizer;
    std::shared_ptr<DiscretePathSearcher_> m_discrete_path_searcher;
    std::shared_ptr<ValidityChecker_> m_validity_checker;

    unsigned int m_maximum_rescaling_count;
    T m_rescaling_duration_multipler;

    StatisticsStorage statistics_storage;
}; // class RLSS
} // namespace rlss

#endif // RLSS_RLSS_HPP