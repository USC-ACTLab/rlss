#ifndef RLSS_SVM_SEQUENCE_HPP
#define RLSS_SVM_SEQUENCE_HPP

#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <deque>
#include <iterator>

namespace rlss {

template<typename T, unsigned int DIM>
class SVMSequence {
public:
    using Hyperplane = rlss::internal::Hyperplane<T, DIM>;
    using Pair = std::pair<T, Hyperplane>;
    SVMSequence(){
        
    }
    ~SVMSequence(){

    }
    bool add(T time, Hyperplane& plane){
        if (sequence.empty() || sequence.front()->first < time){
            sequence.push_back(std::make_pair(time, plane));
            return true;
        }
        else return false;
    }
    bool forget(T _t){
        if (sequence.empty()) return false;
        else{
            while (sequence.front()->first < _t){
                sequece.pop_front().second
            }
            return true;
        }
    }
    size_type size() const {
        return sequence.size();
    }
    Hyperplane& operator[](size_type i){
        assert(i < sequence.size());
        return sequence[i].second;
    }
    const Hyperplane& operator[](size_type i) const {
        assert(i < sequence.size());
        return sequence[i].second;
    }
    // implement custom iterator, SVMSequenc::const_iterator
    // following template found: https://gist.github.com/jeetsukumaran/307264
    class const_iterator{
    public:
        typedef const_iterator self_type;
        typedef Hyperplane valueType;
        typedef Hyperplane& reference;
        typedef Hyperplane* pointer;
        typedef std::deque<Pair>::const_iterator base_iterator;
        typedef std::forward_iterator_tag iterator_category;
        typedef int difference_type;
        const_iterator(base_iterator itr):iterator_to_deque(itr) {}
        self_type operator++(){++iterator_to_deque; return *this;}
        self_type operator++(int index){iterator_to_deque += index; return *this;}
        const reference operator*() {return iterator_to_queue.second;}
        const pointer operator->() {return iterator_to_queue.second;}
        bool operator==(const self_type& rhs) {return iterator_to_deque == rhs.iterator_to_deque;}
        bool operator!=(const self_type& rhs) {return iterator_to_deque != rhs.iterator_to_deque;}
    private:
        base_iterator iterator_to_deque;
    }
    const_iterator cbegin() const {
        return const_iterator(sequence.cbegin());
    }
    const_iterator cend() const {
        return const_iterator(sequence.cend());
    }
private:
    std::deque<Pair> sequence;
} // class SVMSequence

} // namespace rlss

#endif // RLSS_SVM_SEQUENCE_HPP