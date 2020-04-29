#ifndef RLSS_OCCUPANCY_GRID_HPP
#define RLSS_OCCUPANCY_GRID_HPP

#include <unordered_set>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <absl/strings/str_cat.h>
#include <queue>
#include <Eigen/StdVector>

namespace rlss {

template<typename T, unsigned int DIM>
class OccupancyGrid {
    using VectorDIM = Eigen::Vector<T, DIM>;
    using VectorlliDIM = Eigen::Vector<long long int, DIM>; 
    using AlignedBox = Eigen::AlignedBox<T, DIM>;

    using Coordinate = VectorDIM;
    using Index = VectorlliDIM;

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
        return idx
    }

    Coordinate getCenter(const Index& idx) const {
        Coordinate center;
        for(unsigned int d = 0; d < DIM; d++) {
            center(d) = (idx(d) + 0.5) * m_step_size(d);
        }
        return center;
    }

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
        occupied_indexes.push(min);
        while(!occupied_indexes.empty()) {
            const Index& occ = occupied_indexes.front();
            if(!this->isOccupied(occ)) {
                filled_indexes.push_back(occ);
                this->setOccupancy(occ);
            }

            for(unsigned int d = 0; d < DIM; d++) {
                occ(d)++;
                if(occ(d) <= max(d)) {
                    occupied_indexes.push(occ);
                }
                occ(d)--;
            }
            occupied_indexes.pop();
        }

        return filled_indexes;
    }

    std::vector<Index> fillOccupancy(const Coordinate& min, const Coordinate& max) {
        return this->fillOccupancy(this->getIndex(min), this->getIndex(max));
    }

    void isOccupied(const Index& idx) const {
        return m_grid.find(idx) != m_grid.end();
    }

    void isOccupied(const Coordinate& coord) const {
        return this->isOccupied(this->getIndex(coord));
    }

    void addObstacle(const AlignedBox& box) {
        this->fillOccupancy(box.min(), box.max());
    }

    void addTemporaryObstacle(const AlignedBox& box) {
        std::vector<Index> newly_filled 
            = this->fillOccupancy(box.min(), box.max());

        m_temporary_occupancy.insert(newly_filled.begin(), newly_filled.end());
    }

    void clearTemporaryObstacles() {
        for(const auto& idx : m_temporary_occupancy) {
            m_grid.erase(idx);
        }
        m_temporary_occupancy.clear();
    }

    AlignedBox toBox(const Index& idx) const {
        Coordinate center = this->getCenter(idx);
        return AlignedBox(center - 0.5 * m_step_size, 
                          center + 0.5 * m_step_size);
    }

    AlignedBox toBox(const Coordinate& coord) const {
        return this->toBox(this->getIndex(coord));
    }


private:
    Coordinate m_step_size;
    std::unordered_set<Index> m_grid;

    std::vector<Index> m_temporary_occupancy;
}; // class OccupancyGrid

} // namespace rlss

#endif // RLSS_OCCUPANCY_GRID_HPP