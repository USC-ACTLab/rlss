#ifndef RLSS_OCCUPANCY_GRID_HPP
#define RLSS_OCCUPANCY_GRID_HPP

#include <unordered_set>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <absl/strings/str_cat.h>
#include <queue>
#include <Eigen/StdVector>
#include <boost/functional/hash/hash.hpp>
#include <rlss/internal/Util.hpp>

namespace rlss {

template<typename T, unsigned int DIM>
class OccupancyGrid {
public:
    using VectorDIM = Eigen::Vector<T, DIM>;
    using VectorlliDIM = Eigen::Vector<long long int, DIM>; 
    using AlignedBox = Eigen::AlignedBox<T, DIM>;
    using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<T, DIM>;

    using Coordinate = VectorDIM;
    using Index = VectorlliDIM;

    struct IndexHasher {
        std::size_t operator()(const Index& idx) const noexcept {
            std::size_t seed = 0;
            for(unsigned int d = 0; d < DIM; d++) {
                boost::hash_combine(seed, idx(d));
            }
            return seed;
        }
    };

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
        occupied_indexes.push(min);
        while(!occupied_indexes.empty()) {
            Index& occ = occupied_indexes.front();
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
        Index min = this->getIndex(box.min());
        Index max = this->getIndex(box.max());

        std::queue<Index> indexes;
        indexes.push(min);
        while(!indexes.empty()) {
            Index& occ = indexes.front();
            if(this->isOccupied(occ))
                return true;

            for(unsigned int d = 0; d < DIM; d++) {
                occ(d)++;
                if(occ(d) <= max(d)) {
                    indexes.push(occ);
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


private:
    Coordinate m_step_size;
    std::unordered_set<Index, IndexHasher> m_grid;

    std::vector<AlignedBox> m_temporary_obstacles;
}; // class OccupancyGrid

} // namespace rlss

#endif // RLSS_OCCUPANCY_GRID_HPP