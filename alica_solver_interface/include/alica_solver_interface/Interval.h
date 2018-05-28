#pragma once

#include <algorithm>
#include <assert.h>
#include <ostream>
namespace alica
{
template <typename T>
class Interval
{
  public:
    Interval() {}
    Interval(T min, T max)
        : _min(min)
        , _max(max)
    {
    }

    Interval<T>(const Interval<T>& o) = default;
    Interval<T>& operator=(const Interval<T>& o) = default;

    T getMin() const { return _min; }
    T getMidPoint() const { return (_max + _min) / 2; }
    T getMax() const { return _max; }
    void limitTo(T min, T max)
    {
        _min = std::max<T>(min, _min);
        _max = std::min<T>(max, _max);
    }
    void limitTo(const Interval<T> o) { limitTo(o._min, o._max); }

    Interval intersect(const Interval<T> o) const { return Interval(std::max<T>(_min, o._min), std::min<T>(_max, o._max)); }
    bool contains(const Interval<T> o) const { return _min <= o._min && _max >= o._max; }
    T clamp(T val) const
    {
        assert(isValid());
        return std::max<T>(_min, std::min<T>(val, _max));
    }

    bool isValid() const { return _min <= _max; }
    T size() const { return _max - _min; }

    void setMin(T m) { _min = m; }
    void setMax(T m) { _max = m; }

  private:
    T _min;
    T _max;
};

template <typename T>
std::ostream& operator<<(std::ostream& out, const Interval<T> c)
{
    out << "[" << c.getMin() << ", " << c.getMax() << "]";
    return out;
}
}
