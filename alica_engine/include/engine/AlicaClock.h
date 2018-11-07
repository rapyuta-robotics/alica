#pragma once

#include <cstdint>
#include <iostream>

namespace alica
{

class AlicaTime
{
public:
    constexpr AlicaTime()
            : _time(0LL)
    {
    }

    constexpr int64_t inNanoseconds() const { return _time; }
    constexpr int64_t inMicroseconds() const { return _time / 1000LL; }
    constexpr int64_t inMilliseconds() const { return _time / 1000000LL; }
    constexpr int64_t inSeconds() const { return _time / 1000000000LL; }
    constexpr int64_t inMinutes() const { return _time / (1000000000LL * 60LL); }
    constexpr int64_t inHours() const { return _time / (1000000000LL * 60LL * 60LL); }

    static constexpr AlicaTime zero() { return AlicaTime(); }

    template <typename T>
    static constexpr AlicaTime nanoseconds(T n)
    {
        return AlicaTime(n);
    }

    template <typename T>
    static constexpr AlicaTime microseconds(T n)
    {
        return AlicaTime(1000LL * n);
    }

    template <typename T>
    static constexpr AlicaTime milliseconds(T n)
    {
        return AlicaTime(1000000LL * n);
    }

    template <typename T>
    static constexpr AlicaTime seconds(T n)
    {
        return AlicaTime(1000000000LL * n);
    }

    template <typename T>
    static constexpr AlicaTime minutes(T n)
    {
        return AlicaTime(1000000000LL * 60LL * n);
    }

    template <typename T>
    static constexpr AlicaTime hours(T n)
    {
        return AlicaTime(1000000000LL * 60LL * 60LL * n);
    }

    AlicaTime abs() { return AlicaTime(std::abs(_time)); }

    constexpr AlicaTime operator+(const AlicaTime& t) const { return AlicaTime(_time + t.inNanoseconds()); }

    constexpr AlicaTime operator-(const AlicaTime& t) const { return AlicaTime(_time - t.inNanoseconds()); }

    AlicaTime& operator-=(const AlicaTime& t)
    {
        _time -= t.inNanoseconds();
        return *this;
    }

    AlicaTime& operator+=(const AlicaTime& t)
    {
        _time += t.inNanoseconds();
        return *this;
    }

    template <typename T>
    constexpr AlicaTime operator/(T t) const
    {
        return AlicaTime(_time / t);
    }

    template <typename T>
    constexpr AlicaTime operator*(T t) const
    {
        return AlicaTime(_time * t);
    }

    template <typename T>
    AlicaTime& operator/=(T t)
    {
        _time /= t;
        return *this;
    }

    template <typename T>
    AlicaTime& operator*=(T t)
    {
        _time *= t;
        return *this;
    }

    constexpr bool operator<(const AlicaTime& t) const { return _time < t.inNanoseconds(); }

    constexpr bool operator<=(const AlicaTime& t) const { return _time <= t.inNanoseconds(); }

    constexpr bool operator>(const AlicaTime& t) const { return _time > t.inNanoseconds(); }

    constexpr bool operator>=(const AlicaTime& t) const { return _time >= t.inNanoseconds(); }

    constexpr bool operator==(const AlicaTime& t) const { return _time == t.inNanoseconds(); }

    constexpr bool operator!=(const AlicaTime& t) const { return _time != t.inNanoseconds(); }

    friend std::ostream& operator<<(std::ostream& os, const AlicaTime& time);

private:
    template <typename T>
    constexpr explicit AlicaTime(T t)
            : _time(t)
    {
    }

    int64_t _time;
};

class AlicaClock
{
public:
    AlicaClock() {}
    virtual ~AlicaClock() {}
    virtual AlicaTime now() const;
    void sleep(const AlicaTime&);
};

} // namespace alica