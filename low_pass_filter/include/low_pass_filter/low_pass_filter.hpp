#ifndef LOW_PASS_FILTER_HPP_
#define LOW_PASS_FILTER_HPP_

const double PI = 3.14159265359;

template <typename T>
class LowPassFilter
{
public:
    LowPassFilter(double ts, double cut_off_frequency, T pre_val);
    virtual ~LowPassFilter();

    T process(const T & val);

private:
    double ts_;
    double cut_off_frequency_;
    double tau_;
    T pre_val_;
};

template <typename T>
LowPassFilter<T>::LowPassFilter(double ts, double cut_off_frequency, T pre_val) :
    ts_(ts), cut_off_frequency_(cut_off_frequency), pre_val_(pre_val)
{
    tau_ = 1 / (2 * PI * cut_off_frequency_);
}

template <typename T>
LowPassFilter<T>::~LowPassFilter()
{
}

template <typename T>
T LowPassFilter<T>::process(const T & val)
{
    pre_val_ = (ts_ * val + tau_ * pre_val_) / (tau_ + ts_);

    return pre_val_;
}

#endif
