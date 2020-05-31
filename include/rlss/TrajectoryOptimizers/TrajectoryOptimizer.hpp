#ifndef RLSS_TRAJECTORY_OPTIMIZER_HPP
#define RLSS_TRAJECTORY_OPTIMIZER_HPP

#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>

namespace rlss {

template<typename T, unsigned int DIM>
class TrajectoryOptimizer {
public:
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<T, DIM>;
    using AlignedBox = rlss::internal::AlignedBox<T, DIM>;
    using OccupancyGrid = rlss::OccupancyGrid<T, DIM>;
    using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;

    virtual ~TrajectoryOptimizer() {

    }

    // returns std::nullopt when optimization fails
    virtual std::optional<PiecewiseCurve> optimize(
        const StdVectorVectorDIM& segments,
        const std::vector<T>& durations,
        const std::vector<AlignedBox>& oth_rbt_col_shape_bboxes,
        const OccupancyGrid& occupancy_grid,
        const StdVectorVectorDIM& current_robot_state
    ) = 0;

}; // class TrajectoryOptimizer

} // namespace rlss

#endif // RLSS_TRAJECTORY_OPTIMIZER_HPP