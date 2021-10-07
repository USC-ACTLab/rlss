#ifndef RLSS_SVM_SEQUENCE_HPP
#define RLSS_SVM_SEQUENCE_HPP

#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <deque>

namespace rlss {

template<typename T, unsigned int DIM>
class SVMSequence {
    private:
        T current_time;
        T start_time;
        
    public:
        SVMSequence(T start){
        }

    ~SVMSequence(){

    }
    
    bool add() = 0;
    bool forget() = 0;

} // class SVMSequence

} // namespace rlss

#endif // RLSS_SVM_SEQUENCE_HPP