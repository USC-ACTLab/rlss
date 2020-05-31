#ifndef RLSS_RLSS_VALIDITY_CHECKER_HPP
#define RLSS_RLSS_VALIDITY_CHECKER_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/ValidityCheckers/ValidityChecker.hpp>

namespace rlss {
template<typename T, unsigned int DIM>
class RLSSValidityChecker: public ValidityChecker<T, DIM> {
public:
    using Base = ValidityChecker<T, DIM>;
    using PiecewiseCurve = typename Base::PiecewiseCurve;

    RLSSValidityChecker(
        const std::vector<std::pair<unsigned int, T>>& maxdermag,
        T search_step
    ): m_max_derivative_magnitudes(maxdermag),
       m_search_step(search_step)
    {

    }

    bool isValid(const PiecewiseCurve &curve) override {
        for(const auto& [d, l]: m_max_derivative_magnitudes) {
            for(
                    T param = 0;
                    param < curve.maxParameter();
                    param += m_search_step
                    ) {
                T norm = curve.eval(param, d).norm();
                if(norm > l) {
                    debug_message(
                            internal::debug::colors::RED,
                            "norm of the ",
                            d,
                            "th degree of curve's derivative at ",
                            param,
                            " is ",
                            norm,
                            " while the maximum allowed is ",
                            l,
                            internal::debug::colors::RESET
                    );
                    return false;
                }
            }
        }
        return true;
    }


private:
    std::vector<std::pair<unsigned int, T>> m_max_derivative_magnitudes;
    T m_search_step;
};
} // namespace rlss
#endif // RLSS_RLSS_VALIDITY_CHECKER_HPP