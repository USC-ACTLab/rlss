#ifndef RLSS_VALIDITY_CHECKER_HPP
#define RLSS_VALIDITY_CHECKER_HPP

#include <splx/curve/PiecewiseCurve.hpp>

namespace rlss {

template<typename T, unsigned int DIM>
class ValidityChecker {
public:
    using PiecewiseCurve = splx::PiecewiseCurve<T, DIM>;

    virtual ~ValidityChecker() {

    }

    virtual bool isValid(const PiecewiseCurve &curve) = 0;
}; // class ValidityChecker

} // namespace rlss
#endif // RLSS_VALIDITY_CHECKER_HPP