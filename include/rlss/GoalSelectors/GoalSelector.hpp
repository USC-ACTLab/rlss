#ifndef RLSS_GOAL_SELECTOR_HPP
#define RLSS_GOAL_SELECTOR_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/OccupancyGrid.hpp>
namespace rlss {

template<typename T, unsigned int DIM>
class GoalSelector {
public:
    using VectorDIM = rlss::internal::VectorDIM<T, DIM>;
    using OccupancyGrid = rlss::OccupancyGrid<T, DIM>;

    virtual ~GoalSelector() {

    }

    virtual std::optional<std::pair<VectorDIM, T>> select
            (
                    const VectorDIM &current_position,
                    const OccupancyGrid &occupancy_grid,
                    T current_time
            ) = 0;

};

} // namespace rlss
#endif // RLSS_GOAL_SELECTOR_HPP