#ifndef RLSS_INTERNAL_UTIL_HPP
#define RLSS_INTERNAL_UTIL_HPP

#include <Eigen/Geometry>
#include <boost/functional/hash/hash_fwd.hpp>
#include <memory>
#include <Eigen/Dense>
#include <queue>
#include <functional>
#include <stdexcept>
#include <absl/strings/str_cat.h>

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


template <typename T, unsigned int DIM>
StdVectorVectorDIM<T, DIM> linearInterpolate(
    const VectorDIM<T, DIM>& start,
    const VectorDIM<T, DIM>& end,
    std::size_t num_points
) {

    using StdVectorVectorDIM = StdVectorVectorDIM<T, DIM>;
    using VectorDIM = VectorDIM<T, DIM>;

    if(num_points < 2) {
        throw std::domain_error(
            absl::StrCat
            (
                "linear interpolate can't have less than 2 number of points",
                " given: ",
                num_points
            )
        );
    }

    StdVectorVectorDIM result(num_points);

    VectorDIM step_vec = (end - start) / (num_points - 1);

    result.push_back(start);
    for(std::size_t step = 1; step < num_points - 1; step++) {
        result.push_back(start + step * step_vec);
    }
    result.push_back(end);

    return result;


}

template <typename T, unsigned int DIM>
StdVectorVectorDIM<T, DIM> bestSplitSegments(
    const StdVectorVectorDIM<T, DIM>& segments,
    std::size_t num_pieces
) {
    if(num_pieces + 1 < segments.size()) {
        throw std::domain_error(
            absl::StrCat
            (
                "nothing to split since num_pieces=",
                num_pieces,
                "is less than current number of segments ",
                segments.size() - 1
            )
        );
    }


    using VectorDIM = VectorDIM<T, DIM>;
    using StdVectorVectorDIM = StdVectorVectorDIM<T, DIM>;

    if(segments.size() == 1) {
        return StdVectorVectorDIM(num_pieces + 1, segments[0]);
    }

    struct SegmentPQElem {
        VectorDIM start;
        VectorDIM end;
        T length;
        unsigned int num_pieces;
        std::size_t segment_idx;

        SegmentPQElem(const VectorDIM& s, const VectorDIM& e, std::size_t i)
                :   start(s),
                    end(e),
                    length((s-e).norm()),
                    num_pieces(1),
                    segment_idx(i)
        {

        }

        bool operator<(const SegmentPQElem& rhs) const {
            return this->length / this->num_pieces
                        > rhs.length / rhs.num_pieces;
        }
    };

    std::priority_queue
    <
        SegmentPQElem,
        std::vector<SegmentPQElem>
    > pq;

    for(std::size_t i = 0; i < segments.size()-1; i++) {
        pq.push({segments[i], segments[i+1], i});
    }

    for(std::size_t p_count = segments.size() - 1;
            p_count < num_pieces; p_count++) {
        SegmentPQElem t = pq.top();
        pq.pop();
        t.num_pieces++;
        pq.push(t);
    }

    std::vector<SegmentPQElem> segment_pqelems;
    while(!pq.empty()) {
        segment_pqelems.push_back(pq.top());
        pq.pop();
    }

    std::sort(
            segment_pqelems.begin(),
            segment_pqelems.end(),
            [] (const SegmentPQElem& lhs, const SegmentPQElem& rhs) -> bool {
                return lhs.segment_idx < rhs.segment_idx;
            }
    );

    StdVectorVectorDIM result;

    for(const SegmentPQElem& pqelem: segment_pqelems) {
        StdVectorVectorDIM interpolate
            = linearInterpolate<T, DIM>(
                    pqelem.start,
                    pqelem.end,
                    pqelem.num_pieces + 1
        );

        result.insert(result.end(), interpolate.begin(), interpolate.end() - 1);
    }

    result.push_back(segments.back());
    return  result;
}

// shift hyperplane hp creating hyperplane shp
// such that whenever the center_of_mass of the robot
// is to the negative side of the shp,
// the collision shape box of the
// robot is to the negative side of the hyperplane hp.
// box should be the bounding box of the collision shape at given
// center_of_mass
template<typename T, unsigned int DIM>
Hyperplane<T, DIM> shiftHyperplane(
    const VectorDIM<T, DIM>& center_of_mass,
    const AlignedBox<T, DIM>& box,
    const Hyperplane<T, DIM>& hp
) {
    using Hyperplane = Hyperplane<T, DIM>;
    using StdVectorVectorDIM = StdVectorVectorDIM<T, DIM>;
    Hyperplane shp {hp.normal(), hp.offset()};

    T offset = std::numeric_limits<T>::max();

    StdVectorVectorDIM corner_points = cornerPoints<T, DIM>(box);

    for(const auto& pt : corner_points) {
        shp.offset()
                = std::min(offset, hp.normal().dot(center_of_mass - pt));
    }

    return shp;
}

/*
* Buffer 'box' so that when center_of_mass is inside the buffered box
* com_box corresponding to robot with center of mass 'center_of_mass'
* is inside the 'box'
*/
template<typename T, unsigned int DIM>
AlignedBox<T, DIM> bufferAlignedBox(
    const VectorDIM<T, DIM>& center_of_mass,
    const AlignedBox<T, DIM>& com_box,
    const AlignedBox<T, DIM>& box
) {
    return AlignedBox<T, DIM>(box.min() + (center_of_mass - com_box.min()),
                      box.max() + (center_of_mass - com_box.max()));
}

} // namespace internal

} // namespace rlss

#endif // RLSS_INTERNAL_UTIL_HPP