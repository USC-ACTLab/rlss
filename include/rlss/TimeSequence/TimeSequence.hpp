#ifndef RLSS_TIME_SEQUENCE_HPP
#define RLSS_TIME_SEQUENCE_HPP

#include <rlss/internal/Util.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <deque>
#include <iterator>
#include <functional>
#include <absl/status/statusor.h>
#include <math.h>

namespace rlss {

// time comparator = std:greater/less
template<typename TimeType, typename ElementType, typename Comparator = std::less<TimeType>>
// TimeSequence
// comparator for time to decide ascending/descending
class TimeSequence {
    using Pair = std::pair<TimeType, ElementType>;
    using StatusOrRef = absl::StatusOr<std::reference_wrapper<const ElementType>>;
public:
    TimeSequence() {}
    ~TimeSequence() {}

    bool compare(TimeType a, TimeType b, Comparator comp = Comparator()){
        return comp(a, b);
    }

    // return if added to sequence at all
    bool add(TimeType time, const ElementType& item){
        // consider input NaN
        // reference: https://stackoverflow.com/questions/570669/checking-if-a-double-or-float-is-nan-in-c
        if (time != time) return false;
        if (sequence.empty() || compare(sequence.back().first, time)){
            sequence.push_back(std::make_pair(time, item));
            return true;
        }
        return false;
    }

    // return how many items popped
    size_t forget(TimeType time){
        // consider input NaN
        if (time != time) return 0;
        size_t counter = 0;
        while (!sequence.empty() && compare(sequence.front().first, time)){
            sequence.pop_front();
            counter++;
        }
        return counter;
    }

    size_t size() const {return sequence.size();}
    bool empty() {return sequence.empty();}

    StatusOrRef operator[](size_t i){
        if (i >= sequence.size()){
            return absl::InvalidArgumentError("index out of range");
        }
        return sequence[i].second;
    }

    // implement custom iterator, TimeSequence::const_iterator
    // following template found: https://gist.github.com/jeetsukumaran/307264
    class const_iterator{
        typedef const_iterator self_type;
        typedef ElementType valueType;
        typedef const ElementType& reference;
        typedef const ElementType* pointer;
        typedef typename std::deque<Pair>::const_iterator base_iterator;
        typedef std::forward_iterator_tag iterator_category;
        typedef int difference_type;
    public:
        const_iterator(base_iterator itr):iterator_to_deque(itr) {}
        self_type operator++(){++iterator_to_deque; return *this;}
        self_type operator++(int ignored){++iterator_to_deque; return *this;}
        reference operator*() const {return (iterator_to_deque->second);}
        pointer operator->() const {return &(iterator_to_deque->second);}
        bool operator==(const self_type& rhs) const {return iterator_to_deque == rhs.iterator_to_deque;}
        bool operator!=(const self_type& rhs) const {return iterator_to_deque != rhs.iterator_to_deque;}
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
}; // class TimeSequence

} // namespace rlss

#endif // RLSS_TIME_SEQUENCE_HPP