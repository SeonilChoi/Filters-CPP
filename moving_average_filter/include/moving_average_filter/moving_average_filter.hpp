#ifndef MOVING_AVERAGE_FILTER_HPP_
#define MOVING_AVERAGE_FILTER_HPP_

#include <vector>
#include <memory>

template <typename T>
class MovingAverageFilter
{
public:
    MovingAverageFilter(size_t length, T sum);
    virtual ~MovingAverageFilter();

    T process(const T & val);

private:
    size_t length_;
    T sum_;
    std::vector<T> list_;
};

template <typename T>
MovingAverageFilter<T>::MovingAverageFilter(size_t length, T sum) :
    length_(length), sum_(sum)
{
    list_.reserve(length_);
}

template <typename T>
MovingAverageFilter<T>::~MovingAverageFilter()
{
    list_.clear();
}

template <typename T>
T MovingAverageFilter<T>::process(const T & val)
{
    if (list_.size() == length_){
        sum_ -= list_.front();
        list_.erase(list_.begin());
    }
    list_.push_back(val);
    sum_ += val;

    return sum_ / list_.size();
}

#endif
