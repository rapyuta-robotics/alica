#pragma once

#include <atomic>

namespace alica
{
class CounterClass
{
public:
    CounterClass();
    virtual ~CounterClass();
    static std::atomic<int> called;
};
} // namespace alica
