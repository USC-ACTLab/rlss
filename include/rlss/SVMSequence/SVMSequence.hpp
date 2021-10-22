#ifndef RLSS_SVM_SEQUENCE_HPP
#define RLSS_SVM_SEQUENCE_HPP

#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <deque>
#include <iterator>

namespace rlss {

template<typename T, unsigned int DIM>
class SVMSequence {
    using Hyperplane = rlss::internal::Hyperplane<T, DIM>;
    using Pair = std::pair<T, Hyperplane>;
public:
    SVMSequence() {}
    ~SVMSequence() {}

    // return if added to sequence at all
    bool add(T time, const Hyperplane& plane){
        if (sequence.empty() || sequence.front().first < time){
            sequence.push_back(std::make_pair(time, plane));
            return true;
        }
        return false;
    }

    // return how many items popped
    size_t forget(T _t){
        size_t counter = 0;
        while (!sequence.empty() && sequence.front().first < _t){
            sequence.pop_front();
            counter++;
        }
        return counter;
    }

    size_t size() const {return sequence.size();}
    bool empty() {return sequence.empty();}

    Hyperplane& operator[](size_t i){
        assert(i < sequence.size());
        return sequence[i].second;
    }

    const Hyperplane& operator[](size_t i) const {
        assert(i < sequence.size());
        return sequence[i].second;
    }
    // implement custom iterator, SVMSequenc::const_iterator
    // following template found: https://gist.github.com/jeetsukumaran/307264
    class const_iterator{
        typedef const_iterator self_type;
        typedef Hyperplane valueType;
        typedef Hyperplane& reference;
        typedef Hyperplane* pointer;
        typedef typename std::deque<Pair>::const_iterator base_iterator;
        typedef std::forward_iterator_tag iterator_category;
        typedef int difference_type;
    public:
        const_iterator(base_iterator itr):iterator_to_deque(itr) {}
        self_type operator++(){++iterator_to_deque; return *this;}
        self_type operator++(int index){iterator_to_deque += index; return *this;}
        const reference operator*() {return iterator_to_deque.second;}
        const pointer operator->() {return iterator_to_deque.second;}
        bool operator==(const self_type& rhs) {return iterator_to_deque == rhs.iterator_to_deque;}
        bool operator!=(const self_type& rhs) {return iterator_to_deque != rhs.iterator_to_deque;}
    private:
        base_iterator iterator_to_deque;
    };
    const_iterator cbegin() const {
        return const_iterator(sequence.cbegin());
    }
    const_iterator cend() const {
        return const_iterator(sequence.cend());
    }
private:
    std::deque<Pair> sequence;
}; // class SVMSequence

} // namespace rlss

#endif // RLSS_SVM_SEQUENCE_HPP