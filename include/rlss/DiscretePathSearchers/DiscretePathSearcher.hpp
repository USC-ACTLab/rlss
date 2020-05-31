#ifndef RLSS_DISCRETE_PATH_SEARCHER_HPP
#define RLSS_DISCRETE_PATH_SEARCHER_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/OccupancyGrid.hpp>

namespace rlss {
template<typename T, unsigned int DIM>
class DiscretePathSearcher {
public:

    using VectorDIM = rlss::internal::VectorDIM<T, DIM>;
    using OccupancyGrid = rlss::OccupancyGrid<T, DIM>;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<T, DIM>;

    virtual ~DiscretePathSearcher() {

    }

    virtual
    std::optional<std::pair<StdVectorVectorDIM, std::vector<T>>>
    search(
            const VectorDIM& current_position,
            const VectorDIM& goal_position,
            T time_horizon,
            const OccupancyGrid& occupancy_grid
    ) = 0;

};
}

#endif // RLSS_DISCRETE_PATH_SEARCHER_HPP