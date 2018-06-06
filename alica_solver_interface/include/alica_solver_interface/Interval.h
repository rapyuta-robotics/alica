#pragma once

#include <algorithm>
#include <alica_common_config/common_defines.h>
#include <assert.h>
#include <ostream>

namespace alica
{
template <typename T>
class Interval
{
public:
    constexpr Interval() {}
    constexpr Interval(T min, T max)
            : _min(min)
            , _max(max)
    {
    }

    Interval<T>(const Interval<T>& o) = default;
    Interval<T>& operator=(const Interval<T>& o) = default;

    constexpr T getMin() const { return _min; }
    constexpr T getMidPoint() const { return (_max + _min) / 2; }
    constexpr T getMax() const { return _max; }
    void intersect(T min, T max)
    {
        _min = std::max<T>(min, _min);
        _max = std::min<T>(max, _max);
    }
    void intersect(const Interval<T> o) { intersect(o._min, o._max); }

    constexpr Interval intersection(const Interval<T> o) const { return Interval(std::max<T>(_min, o._min), std::min<T>(_max, o._max)); }
    constexpr bool contains(const Interval<T> o) const { return _min <= o._min && _max >= o._max; }
    constexpr T clamp(T val) const { return ALICA_ASSERT(isValid()), std::max<T>(_min, std::min<T>(val, _max)); }

    constexpr bool isValid() const { return _min <= _max; }
    constexpr T size() const { return _max - _min; }

    void setMin(T m) { _min = m; }
    void setMax(T m) { _max = m; }

    constexpr bool operator==(const Interval<T> o) const { return _min == o._min && _max == o._max; }
    constexpr bool operator!=(const Interval<T> o) const { return !(*this == o); }

    Interval<T>& operator/=(const T v)
    {
        *this = *this / v;
        return *this;
    }
    Interval<T>& operator*=(const T v)
    {
        *this = *this * v;
        return *this;
    }
    Interval<T>& operator+=(const Interval<T> v)
    {
        *this = *this + v;
        return *this;
    }
    Interval<T>& operator-=(const Interval<T> v)
    {
        *this = *this - v;
        return *this;
    }

    Interval<T>& operator++()
    {
        ++_min;
        ++_max;
        return *this;
    }
    Interval<T>& operator--()
    {
        --_min;
        --_max;
        return *this;
    }

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

template <typename T>
constexpr Interval<T> operator+(const Interval<T> a, const Interval<T> b)
{
    return Interval<T>(a._min + b._min, a._max + b._max);
}
template <typename T>
constexpr Interval<T> operator-(const Interval<T> a, const Interval<T> b)
{
    return Interval<T>(a._min - b._max, a._max - b._min);
}

template <typename T>
constexpr Interval<T> operator*(const Interval<T> a, T b)
{
    return Interval<T>(a._min * b, a._max * b);
}
template <typename T>
constexpr Interval<T> operator*(T b, const Interval<T> a)
{
    return a * b;
}
template <typename T>
constexpr Interval<T> operator/(const Interval<T> a, T b)
{
    return Interval<T>(a._min / b, a._max / b);
}
}
