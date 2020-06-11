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
#include <iostream>
#include <fstream>
#include <qp_wrappers/cplex.hpp>
#include <qp_wrappers/gurobi.hpp>
#include <qp_wrappers/qpoases.hpp>
#include <qp_wrappers/osqp.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <lp_wrappers/cplex.hpp>

#define RLSS_HARD_QP_SOLVER qpOASES
#define RLSS_SOFT_QP_SOLVER CPLEX
#define RLSS_SVM_QP_SOLVER qpOASES
#define RLSS_HP_PRUNING_LP_SOLVER CPLEX

namespace rlss {

#ifdef ENABLE_RLSS_DEBUG_MESSAGES
namespace internal {
    template<typename T>
    void debug_message_internal(bool first, T message) {
        if(first) {
            std::cout << "[DEBUG] ";
        }
        std::cout << message << std::endl;
    }

    template<typename T, typename... Args>
    void debug_message_internal(bool first, T message, Args... args) {
        if(first) {
            std::cout << "[DEBUG] ";
        }
        std::cout << message;
        debug_message_internal(false, args...);
    }
} // namespace internal

    template<typename... Args>
    void debug_message(Args... args) {
        internal::debug_message_internal(true, args...);
    }
#else
    template<typename... Args>
    void debug_message(Args... args) {

    }
#endif

namespace internal {
    namespace debug {
        namespace colors {
            constexpr char RESET[] = "\033[0m";
            constexpr char RED[] = "\033[31m";
            constexpr char GREEN[] = "\033[32m";
        }
    }
}

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

    result[0] = start;
    for(std::size_t step = 1; step < num_points - 1; step++) {
        result[step] = start + step * step_vec;
    }
    result.back() = end;

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
                        < rhs.length / rhs.num_pieces;
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
    Hyperplane shp {hp.normal(), std::numeric_limits<T>::lowest()};


    StdVectorVectorDIM corner_points = cornerPoints<T, DIM>(box);

    for(const auto& pt : corner_points) {
        shp.offset()
                = std::max(shp.offset(),
                           hp.normal().dot(pt - center_of_mass) + hp.offset()
        );
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

/*
 * Prunes hyperplanes h in hps such that if the space S bounded by the bounding
 * box bbox and the hyperplane h contains the space S' bounded by the bounding
 * box bbox and an hyperplane h' in hps (that is S' is a subset of S).
 */
template<typename T, unsigned int DIM>
std::vector<Hyperplane<T, DIM>> pruneHyperplanes(
        const std::vector<Hyperplane<T,DIM>>& hps,
        const AlignedBox<T, DIM>& bbox) {

    using Hyperplane = Hyperplane<T, DIM>;
    using AlignedBox = AlignedBox<T, DIM>;
    using LP = LPWrappers::Problem<T>;
    using LPEngine = LPWrappers::RLSS_HP_PRUNING_LP_SOLVER::Engine<T>;
    using Vector = typename LPEngine::Vector;

    std::vector<Hyperplane> result;
    LP lp(DIM, 2);
    LPEngine solver;

    for(unsigned int d = 0; d < DIM; d++) {
        lp.set_var_limits(d, bbox.min()(d), bbox.max()(d));
    }

    Vector c(DIM);
    for(unsigned int d = 0; d < DIM; d++) c(d) = 1;
    lp.add_c(c);

    for(const auto& hp_to_remove: hps) {
        bool should_remove = false;
        for(const auto& hp_to_check: hps) {
            lp.set_constraint(0, hp_to_check.normal().transpose(),
                    LP::minus_infinity, hp_to_check.offset());

            lp.set_constraint(1, hp_to_remove.normal().transpose(),
                              hp_to_remove.offset(), LP::infinity);

            Vector soln;
            auto ret = solver.init(lp, soln);
            if(ret != LPWrappers::OptReturnType::Optimal) {
                should_remove = true;
                break;
            }
        }

        if(!should_remove) {
            result.push_back(hp_to_remove);
        }
    }

    return result;
}


} // namespace internal

} // namespace rlss

#endif // RLSS_INTERNAL_UTIL_HPP