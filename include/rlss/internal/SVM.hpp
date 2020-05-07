#ifndef RLSS_INTERNAL_SVM_HPP
#define RLSS_INTERNAL_SVM_HPP
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/cplex.hpp>
#include <rlss/internal/Util.hpp>

namespace rlss {

namespace internal {

/*
* Calculate the svm hyperplane between two set of points f and s
* such that for all points p \in f, np + d < 0 where n is the normal
* of the hyperplane and d is the offset of the hyperplane.
*/
template<typename T, unsigned int DIM>
Hyperplane<T, DIM> svm(
    const StdVectorVectorDIM<T,DIM>& f, 
    const StdVectorVectorDIM<T,DIM>& s) {

    QPWrappers::Problem svm_qp(DIM + 1);
    Matrix Q(DIM+1, DIM+1);
    Q.setIdentity();
    Q(DIM, DIM) = 0;

    svm_qp.add_Q(Q);

    for(const VectorDIM& pt : f) {
        Row<T> coeff(DIM+1);
        coeff.setZero();

        for(unsigned int d = 0; d < DIM; d++) {
            coeff(d) = pt(d);
        }
        coeff(DIM) = 1;

        svm_qp.add_constraint(coeff, std::numeric_limits<T>::lowest(), -1);
    }

    for(const VectorDIM& pt : s) {
        Row<T> coeff(DIM+1);
        coeff.setZero();

        for(unsigned int d = 0; d < DIM; d++) {
            coeff(d) = pt(d);
        }
        coeff(DIM) = 1;

        svm_qp.add_constraint(coeff, 1, std::numeric_limits<T>::max());
    }

    QPWrappers::CPLEX cplex;
    Vector<T> result(DIM+1);
    auto ret = cplex.init(svm_qp, result);
    Hyperplane<T, DIM> hp;

    if(ret == QPWrappers::OptReturnType::Optimal) {
        for(unsigned int d = 0; d < DIM; d++) {
            hp.normal()(d) = result(d);
        }
        hp.offset() = result(DIM);
    } else {
        throw std::runtime_error(
            absl::StrCat(
                "svm not feasible"
            )
        );
    }

    return hp;
}

} // namespace internal
} // namespace rlss
#endif // RLSS_INTERNAL_SVM_HPP