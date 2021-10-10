#ifndef RLSS_SVM_SEQUENCE_HPP
#define RLSS_SVM_SEQUENCE_HPP

#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
// #include <deque>

namespace rlss {

template<typename T, unsigned int DIM>
class SVMSequence {
public:
    using Hyperplane = rlss::internal::Hyperplane<T, DIM>;
    using Pair = std::pair<T, std::shared_ptr<Hyperplane>>;
    SVMSequence(){
        
    }
    ~SVMSequence(){

    }
    
    bool add(T _t, Hyperplane* plane){
        try{
            sequence.push_back(std::make_pair(_t, std::shared_ptr<Hyperplane>(plane)));
            return true;
        } catch(...){
            return false;
        }
    }
    bool forget(T _t){
        try{
            // TODO is it greater or less?
            while (sequence.front()->first < _t){
                sequece.pop_front().second.reset();
            }
            return true;
        } catch(...) {
            return false;
        }
    }

private:
    deque<Pair> sequence;
} // class SVMSequence

} // namespace rlss

#endif // RLSS_SVM_SEQUENCE_HPP