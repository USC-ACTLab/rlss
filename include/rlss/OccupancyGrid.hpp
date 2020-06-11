#ifndef RLSS_OCCUPANCY_GRID_HPP
#define RLSS_OCCUPANCY_GRID_HPP

#include <limits>
#include <unordered_set>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <absl/strings/str_cat.h>
#include <queue>
#include <Eigen/StdVector>
#include <boost/functional/hash/hash.hpp>
#include <rlss/internal/Util.hpp>
#include <rlss/CollisionShapes/CollisionShape.hpp>
#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/cplex.hpp>

namespace rlss {

template<typename T, unsigned int DIM>
class OccupancyGridIterator;

template<typename T, unsigned int DIM>
class OccupancyGridDistanceIterator;

template<typename T, unsigned int DIM>
class OccupancyGrid {
public:
    using VectorDIM = rlss::internal::VectorDIM<T, DIM>;
    using VectorlliDIM = rlss::internal::VectorDIM<long long int, DIM>;
    using AlignedBox = Eigen::AlignedBox<T, DIM>;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<T, DIM>;

    using Coordinate = VectorDIM;
    using Index = VectorlliDIM;

    using iterator = OccupancyGridIterator<T, DIM>;
    using distance_iterator = OccupancyGridDistanceIterator<T, DIM>;

    using UnorderedIndexSet 
            = std::unordered_set<
                    Index, 
                    rlss::internal::VectorDIMHasher<long long int, DIM>
            >;

    OccupancyGrid(Coordinate step_size): m_step_size(step_size) {

    }

    /*
    * gets the cell index that coordinate is in
    */
    Index getIndex(const Coordinate& coord) const {
        Index idx;
        for(unsigned int d = 0; d < DIM; d++) {
            idx(d) = std::llround((coord(d) / m_step_size(d)) - 0.5);
        }
        return idx;
    }

    /*
    * get the center coordinate of the cell with the given index
    */
    Coordinate getCenter(const Index& idx) const {
        Coordinate center;
        for(unsigned int d = 0; d < DIM; d++) {
            center(d) = (idx(d) + 0.5) * m_step_size(d);
        }
        return center;
    }

    /*
     * Get neighbors of the index
     */
    static std::vector<Index> getNeighbors(const Index& idx) {
        std::vector<Index> res;
        Index dir = Index::Zero();

        for(unsigned int d = 0; d < DIM; d++) {
            for(const long long int& i : {-1, 1}) {
                dir(d) = i;
                res.push_back(idx + dir);
            }
            dir(d) = 0;
        }

        return res;
    }

    /*
     * Get neighbors of the index that the coordinate is in
     */
    std::vector<Index> getNeighbors(const Coordinate& coord) const {
        return this->getNeighbors(this->getIndex(coord));
    }

    /*
    * get the center coordinate of the cell that contains the given coordinate
    */
    Coordinate getCenter(const Coordinate& coord) const {
        return this->getCenter(this->getIndex(coord));
    }

    void removeOccupancy(const Index& idx) {
        m_grid.erase(idx);
    }

    void removeOccupancy(const Coordinate& coord) {
        this->removeOccupancy(this->getIndex(coord));
    }

    void setOccupancy(const Index& idx) {
        m_grid.insert(idx);
    }
    
    void setOccupancy(const Coordinate& coord) {
        this->setOccupancy(this->getIndex(coord));
    }

    std::vector<Index> fillOccupancy(const Index& min, const Index& max) {
        for(unsigned int d = 0; d < DIM; d++) {
            if(min(d) > max(d)) {
                throw std::domain_error(
                    absl::StrCat(
                        "minimum index is bigger than index coordinate",
                        " at dimension ",
                        d,
                        "min(d): ",
                        min(d),
                        " max(d): ",
                        max(d)
                    )
                );
            }
        }

        std::vector<Index> filled_indexes;

        std::queue<Index> occupied_indexes;
        UnorderedIndexSet visited;
        occupied_indexes.push(min);
        visited.insert(min);
        while(!occupied_indexes.empty()) {
            Index& occ = occupied_indexes.front();
            if(!this->isOccupied(occ)) {
                filled_indexes.push_back(occ);
                this->setOccupancy(occ);
            }

            for(unsigned int d = 0; d < DIM; d++) {
                occ(d)++;
                if(occ(d) <= max(d) && visited.find(occ) == visited.end()) {
                    occupied_indexes.push(occ);
                    visited.insert(occ);
                }
                occ(d)--;
            }
            occupied_indexes.pop();
        }

        return filled_indexes;
    }

    std::vector<Index> fillOccupancy(
            const Coordinate& min, const Coordinate& max) {
        return this->fillOccupancy(this->getIndex(min), this->getIndex(max));
    }

    bool isOccupied(const Index& idx) const {        
        AlignedBox idxBox = this->toBox(idx);

        for(const auto& bbox: m_temporary_obstacles) {
            if(bbox.intersects(idxBox)) {
                return true;
            }
        }

        return m_grid.find(idx) != m_grid.end();
    }

    bool isOccupied(const Coordinate& coord) const {
        return this->isOccupied(this->getIndex(coord));
    }

    bool isOccupied(const AlignedBox& box) const {
        for(const auto& bbox: m_temporary_obstacles) {
            if(bbox.intersects(box))
                return true;
        }

        Index min = this->getIndex(box.min());
        Index max = this->getIndex(box.max());

        UnorderedIndexSet visited;
        std::queue<Index> indexes;
        indexes.push(min);
        visited.insert(min);
        while(!indexes.empty()) {
            Index& occ = indexes.front();
            if(m_grid.find(occ) != m_grid.end())
                return true;

            for(unsigned int d = 0; d < DIM; d++) {
                occ(d)++;
                if(occ(d) <= max(d) && visited.find(occ) == visited.end()) {
                    indexes.push(occ);
                    visited.insert(occ);
                }
                occ(d)--;
            }
            indexes.pop();
        }

        return false;

    }

    void addObstacle(const AlignedBox& box) {
        this->fillOccupancy(box.min(), box.max());
    }

    // add obstacle that is the convex hull of points in pts
    void addObstacle(const StdVectorVectorDIM& pts) {
        using Row = rlss::internal::Row<T>;

        if(pts.empty())
            return;

        AlignedBox box(pts[0], pts[0]);
        for(const auto& pt: pts) {
            box.extend(pt);
        }

        Index min = this->getIndex(box.min());
        Index max = this->getIndex(box.max());

        QPWrappers::CPLEX::Engine<T> cplex;
        cplex.setFeasibilityTolerance(1e-9);


        std::queue<Index> q;
        UnorderedIndexSet visited;
        q.push(min);
        visited.insert(min);
        int idx = 0;
        while(!q.empty()) {
            Index& fr = q.front();
            AlignedBox box = this->toBox(fr);

            QPWrappers::Problem<T> problem(DIM + pts.size());
            for(unsigned int d = 0; d < DIM; d++) {
                problem.set_var_limits(d, box.min()(d), box.max()(d));
            }
            for(std::size_t i = 0; i < pts.size(); i++) {
                problem.set_var_limits(
                        DIM + i, 0, std::numeric_limits<T>::max());
            }

            Row coeff(DIM + pts.size());
            coeff.setZero();
            for(std::size_t i = 0; i < pts.size(); i++) {
                coeff(DIM + i) = 1;
            }

            problem.add_constraint(coeff, 1, 1);

            for(unsigned int d = 0; d < DIM; d++) {
                coeff.setZero();
                coeff(d) = -1;
                for(std::size_t i = 0; i < pts.size(); i++) {
                    coeff(DIM + i) = pts[i](d);
                }
                problem.add_constraint(coeff, 0, 0);
            }

            typename QPWrappers::Problem<T>::Vector result;
            auto ret = cplex.init(problem, result);
            if(ret == QPWrappers::OptReturnType::Optimal) {
                this->setOccupancy(fr);
            }

            for(unsigned int d = 0; d < DIM; d++) {
                fr(d)++;
                if(fr(d) <= max(d) && visited.find(fr) == visited.end()) {
                    q.push(fr);
                    visited.insert(fr);
                }
                fr(d)--;
            }

            q.pop();
        }
    }

    void addTemporaryObstacle(const AlignedBox& box) {
        m_temporary_obstacles.push_back(box);
    }

    void clearTemporaryObstacles() {
        m_temporary_obstacles.clear();
    }

    AlignedBox toBox(const Index& idx) const {
        Coordinate center = this->getCenter(idx);
        return AlignedBox(center - 0.5 * m_step_size, 
                          center + 0.5 * m_step_size);
    }

    AlignedBox toBox(const Coordinate& coord) const {
        return this->toBox(this->getIndex(coord));
    }

    iterator begin() const {
        return iterator(*this, 0, m_grid.begin());
    }

    iterator end() const {
        return iterator(*this, m_temporary_obstacles.size(), m_grid.end());
    }

    distance_iterator begin(const AlignedBox& box, T max_distance) const {
        return distance_iterator(*this, 0, m_grid.begin(), box, max_distance);
    }

    distance_iterator end(const AlignedBox& box, T max_distance) const {
        return distance_iterator(*this, m_temporary_obstacles.size(),
                m_grid.end(), box, max_distance);
    }

    friend OccupancyGridIterator<T, DIM>;
    friend OccupancyGridDistanceIterator<T, DIM>;

private:
    Coordinate m_step_size;
    UnorderedIndexSet m_grid;

    std::vector<AlignedBox> m_temporary_obstacles;
}; // class OccupancyGrid


/*
 * Iterator for occupied alignedboxes such that the distance of alignedboxes
 * to the given box in the constructor is at most max_distance_to_max
 */
template<typename T, unsigned int DIM>
class OccupancyGridDistanceIterator {
public:
    using _OccupancyGrid = OccupancyGrid<T, DIM>;
    using UnorderedIndexSet = typename _OccupancyGrid::UnorderedIndexSet;
    using AlignedBox = typename _OccupancyGrid::AlignedBox;

    OccupancyGridDistanceIterator(
            const _OccupancyGrid& g,
            std::size_t temp_obs_idx,
            typename UnorderedIndexSet::const_iterator occ_idx_itr,
            const AlignedBox& bx,
            T max_distance_to_box
    ):
            grid(g),
            temporary_obstacles_idx(temp_obs_idx),
            occupied_idx_iterator(occ_idx_itr),
            box(bx),
            max_distance(max_distance_to_box)
    {
        if((temporary_obstacles_idx < grid.m_temporary_obstacles.size()
              || occupied_idx_iterator != grid.m_grid.end())
              && box.exteriorDistance(*(*this)) > max_distance_to_box) {
            operator++();
        }
    }

    explicit OccupancyGridDistanceIterator(
            const _OccupancyGrid& g,
            const AlignedBox& bx,
            T max_distance_to_box):
            rlss::OccupancyGridIterator<T, DIM>(
                    g,
                    0,
                    grid.m_grid.begin(),
                    bx,
                    max_distance_to_box)
    {
    }

    AlignedBox operator*() const {
        if(temporary_obstacles_idx < grid.m_temporary_obstacles.size()) {
//            debug_message("hop");
            return grid.m_temporary_obstacles[temporary_obstacles_idx];
        } else {
//            debug_message("ba");
            return grid.toBox(*occupied_idx_iterator);
        }
    }

    OccupancyGridDistanceIterator<T,DIM>& operator++() {
        do {
            if (temporary_obstacles_idx < grid.m_temporary_obstacles.size()) {
                temporary_obstacles_idx++;
            } else {
                occupied_idx_iterator++;
            }
        } while((temporary_obstacles_idx < grid.m_temporary_obstacles.size()
                 || occupied_idx_iterator != grid.m_grid.end())
                && box.exteriorDistance(*(*this)) > max_distance);

        return *this;
    }

    bool operator==(const OccupancyGridDistanceIterator<T, DIM>& rhs) const {
        return rhs.temporary_obstacles_idx == this->temporary_obstacles_idx
               && rhs.occupied_idx_iterator == this->occupied_idx_iterator;
    }
    bool operator!=(const OccupancyGridDistanceIterator<T, DIM>& rhs) const {
        return !operator==(rhs);
    }

private:

    const _OccupancyGrid& grid;
    AlignedBox box;
    T max_distance;
    std::size_t temporary_obstacles_idx;
    typename UnorderedIndexSet::const_iterator occupied_idx_iterator;
}; // class OccupancyGridDistanceIterator

template<typename T, unsigned int DIM>
class OccupancyGridIterator {
public:
    using _OccupancyGrid = OccupancyGrid<T, DIM>;
    using UnorderedIndexSet = typename _OccupancyGrid::UnorderedIndexSet;
    using AlignedBox = typename _OccupancyGrid::AlignedBox;

    OccupancyGridIterator(
            const _OccupancyGrid& g,
            std::size_t temp_obs_idx,
            typename UnorderedIndexSet::const_iterator occ_idx_itr
    ):
        grid(g),
        temporary_obstacles_idx(temp_obs_idx),
        occupied_idx_iterator(occ_idx_itr)
    {

    }

    explicit OccupancyGridIterator(const _OccupancyGrid& g):
        rlss::OccupancyGridIterator<T, DIM>(
                g,
                0,
                grid.m_grid.begin())
    {
    }

    AlignedBox operator*() const {
        if(temporary_obstacles_idx < grid.m_temporary_obstacles.size()) {
//            debug_message("hop");
            return grid.m_temporary_obstacles[temporary_obstacles_idx];
        } else {
//            debug_message("ba");
            return grid.toBox(*occupied_idx_iterator);
        }
    }

    OccupancyGridIterator<T,DIM>& operator++() {
        if(temporary_obstacles_idx < grid.m_temporary_obstacles.size()) {
            temporary_obstacles_idx++;
        } else {
            occupied_idx_iterator++;
        }
        return *this;
    }

    bool operator==(const OccupancyGridIterator<T, DIM>& rhs) const {
        return rhs.temporary_obstacles_idx == this->temporary_obstacles_idx
            && rhs.occupied_idx_iterator == this->occupied_idx_iterator;
    }
    bool operator!=(const OccupancyGridIterator<T, DIM>& rhs) const {
        return !operator==(rhs);
    }

private:

    const _OccupancyGrid& grid;
    std::size_t temporary_obstacles_idx;
    typename UnorderedIndexSet::const_iterator occupied_idx_iterator;
};

namespace internal {

template<typename T, unsigned int DIM>
bool segmentValid(
    const OccupancyGrid<T, DIM>& grid,
    const AlignedBox<T, DIM>& workspace,
    const AlignedBox<T, DIM>& from_box,
    const AlignedBox<T, DIM>& to_box
) {
    using AlignedBox = AlignedBox<T, DIM>;

    AlignedBox to_box_copy = to_box;
    to_box_copy.extend(from_box);

    return workspace.contains(to_box_copy) && !grid.isOccupied(to_box_copy);
}

template<typename T, unsigned int DIM>
bool segmentValid(
    const OccupancyGrid<T, DIM>& grid,
    const AlignedBox<T, DIM>& workspace,
    const AlignedBox<T, DIM>& from_box,
    const VectorDIM<T, DIM>& to,
    std::shared_ptr<CollisionShape<T, DIM>> collision_shape
) {
    using AlignedBox = AlignedBox<T, DIM>;
    using OccupancyGrid = OccupancyGrid<T, DIM>;

    AlignedBox to_box = collision_shape->boundingBox(to);

    return segmentValid<T, DIM>(
            grid, workspace, from_box, to_box);
}

template<typename T, unsigned int DIM>
bool segmentValid(
        const OccupancyGrid<T, DIM>& grid,
        const AlignedBox<T, DIM>& workspace,
        const AlignedBox<T, DIM>& from_box,
        const typename OccupancyGrid<T, DIM>::Index& to,
        std::shared_ptr<CollisionShape<T, DIM>> collision_shape
) {
    auto to_center = grid.getCenter(to);
    return segmentValid<T, DIM>(
            grid, workspace, from_box, to_center, collision_shape);
}

template<typename T, unsigned int DIM>
bool segmentValid(
    const OccupancyGrid<T, DIM>& grid,
    const AlignedBox<T, DIM>& workspace, 
    const VectorDIM<T, DIM>& from, 
    const VectorDIM<T, DIM>& to,
    std::shared_ptr<CollisionShape<T, DIM>> collision_shape)
{
    using AlignedBox = AlignedBox<T, DIM>;
    using OccupancyGrid = OccupancyGrid<T, DIM>;

    AlignedBox from_box = collision_shape->boundingBox(from);

    return segmentValid<T, DIM>(
            grid, workspace, from_box, to, collision_shape);
}

template<typename T, unsigned int DIM>
bool segmentValid(
    const OccupancyGrid<T, DIM>& grid,
    const AlignedBox<T, DIM>& workspace,
    const VectorDIM<T, DIM>& from,
    const typename OccupancyGrid<T, DIM>::Index& to,
    std::shared_ptr<CollisionShape<T, DIM>> collision_shape
) {
    auto to_center = grid.getCenter(to);
    return segmentValid<T, DIM>(
            grid, workspace, from, to_center, collision_shape);
}

template<typename T, unsigned int DIM>
bool segmentValid(
    const OccupancyGrid<T, DIM>& grid,
    const AlignedBox<T, DIM>& workspace,
    const typename OccupancyGrid<T, DIM>::Index& from,
    const typename OccupancyGrid<T, DIM>::Index& to,
    std::shared_ptr<CollisionShape<T, DIM>> collision_shape
) {
    auto from_center = grid.getCenter(from);
    auto to_center = grid.getCenter(to);
    return segmentValid<T, DIM>(
            grid, workspace, from_center, to_center, collision_shape);
}


}

} // namespace rlss

#endif // RLSS_OCCUPANCY_GRID_HPP