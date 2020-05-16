#ifndef RLSS_INTERNAL_UTIL_HPP
#define RLSS_INTERNAL_UTIL_HPP

#include <Eigen/Geometry>
#include <boost/functional/hash/hash_fwd.hpp>
#include <memory>
#include <Eigen/Dense>

namespace rlss {

namespace internal {

template<typename T, unsigned int DIM>
using AlignedBox = Eigen::AlignedBox<T, DIM>;

template<typename T, unsigned int DIM>
using Hyperplane = Eigen::Hyperplane<T, DIM>;

template<typename T, unsigned int DIM>
using VectorDIM = Eigen::Matrix<T, DIM, 1>;

template<typename T, unsigned int DIM>
using StdVectorVectorDIM = std::vector<VectorDIM<T,DIM>, 
                            Eigen::aligned_allocator<VectorDIM<T,DIM>>>;

template<typename T>
using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template<typename T>
using Row = Eigen::Matrix<T, 1, Eigen::Dynamic>;

template<typename T>
using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template<typename T, unsigned int DIM>
StdVectorVectorDIM<T, DIM> cornerPoints(const AlignedBox<T, DIM>& box) {
    StdVectorVectorDIM<T, DIM> pts(1<<DIM);
    for(unsigned int i = 0; i < (1<<DIM); i++) {
        for(unsigned int d = 0; d < DIM; d++) {
            pts[i](d) = (i & (1<<d)) ? box.min()(d) : box.max()(d);
        }
    }
    return pts;
}

template<typename T, unsigned int DIM>
class VectorDIMHasher {
public:
    std::size_t operator()(const VectorDIM<T, DIM>& vec) const {
        std::size_t seed = 0;
        for(unsigned int d = 0; d < DIM; d++) {
            boost::hash_combine(seed, vec(d));
        }
        return seed;
    }
};




} // namespace internal

} // namespace rlss

#endif // RLSS_INTERNAL_UTIL_HPP