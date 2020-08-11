#ifndef RLSS_INTERNAL_SVM_HPP
#define RLSS_INTERNAL_SVM_HPP
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/cplex.hpp>
#include <rlss/internal/Util.hpp>
#include <absl/strings/str_cat.h>

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

    QPWrappers::Problem<T> svm_qp(DIM + 1);
    Matrix<T> Q(DIM+1, DIM+1);
    Q.setIdentity();
    Q *= 2;
    Q(DIM, DIM) = 0;

    svm_qp.add_Q(Q);

    for(const VectorDIM<T,DIM>& pt : f) {
        Row<T> coeff(DIM+1);
        coeff.setZero();

        for(unsigned int d = 0; d < DIM; d++) {
            coeff(d) = pt(d);
        }
        coeff(DIM) = 1;

        svm_qp.add_constraint(coeff, std::numeric_limits<T>::lowest(), -1);
    }

    for(const VectorDIM<T,DIM>& pt : s) {
        Row<T> coeff(DIM+1);
        coeff.setZero();

        for(unsigned int d = 0; d < DIM; d++) {
            coeff(d) = pt(d);
        }
        coeff(DIM) = 1;

        svm_qp.add_constraint(coeff, 1, std::numeric_limits<T>::max());
    }


    QPWrappers::RLSS_SVM_QP_SOLVER::Engine<T> solver;
    solver.setFeasibilityTolerance(1e-8);
    Vector<T> result(DIM+1);
    auto ret = solver.init(svm_qp, result);
//    debug_message("svm optimization return value is ", ret);
    Hyperplane<T, DIM> hp;

    if(ret == QPWrappers::OptReturnType::Optimal) {
        for(unsigned int d = 0; d < DIM; d++) {
            hp.normal()(d) = result(d);
        }
        hp.offset() = result(DIM);
    } else {
        // cplex seems more reliable for svm
        QPWrappers::CPLEX::Engine<T> solver;
        solver.setFeasibilityTolerance(1e-8);
        auto ret = solver.init(svm_qp, result);
//        debug_message("svm optimization CPLEX return value is ", ret);
        if(ret == QPWrappers::OptReturnType::Optimal) {
            for(unsigned int d = 0; d < DIM; d++) {
                hp.normal()(d) = result(d);
            }
            hp.offset() = result(DIM);
        } else {
            debug_message("svm failed");
            for(const auto& vec: f) {
                debug_message("f", vec.transpose());
            }
            for(const auto& vec: s) {
                debug_message("s", vec.transpose());
            }
            throw std::runtime_error(
                absl::StrCat(
                        "svm not feasible"
                )
            );
        }
    }

    return hp;
}

} // namespace internal
} // namespace rlss
#endif // RLSS_INTERNAL_SVM_HPP