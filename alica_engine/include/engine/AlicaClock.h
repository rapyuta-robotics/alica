#pragma once

#include <cstdint>
#include <chrono>

namespace alica {

class AlicaTime {
public:

    using nanoseconds = std::chrono::nanoseconds;
    using microseconds = std::chrono::microseconds;
    using milliseconds = std::chrono::milliseconds;
    using seconds = std::chrono::seconds;
    using minutes = std::chrono::minutes;
    using hours = std::chrono::hours;

    template <typename T, typename U, typename = typename std::enable_if<std::is_arithmetic<T>::value && std::is_same<U, nanoseconds>::value, T>::type
    AlicaTime(T time) : _time(time) {}

    AlicaTime<milliseconds> ms(500);
    AlicaTime<seconds> s(4);

    AlicaTime();
    ~AlicaTime();
    // TODO Use compare, arithmetic operators
    // TODO Keep the current internal precision (10000 == 1 ms)
    // TODO Unit tests
    // TODO Add accessors that provide miliseconds, seconds and higher units of time.
    // TODO Constexpr
    // TODO Make current usage consistent
    template

private:
    int64_t _time;
};

class AlicaClock {
public:
    AlicaClock();
    virtual ~AlicaClock();
    virtual AlicaTime now();
    void sleep(long us);
};

}  // namespace alica
