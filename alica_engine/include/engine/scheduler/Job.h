#pragma once

#include <iostream>
#include <chrono>
#include <ctime>
#include <vector>
#include <memory>

namespace alica {
namespace scheduler
{
struct Job
{
    std::function<void()> cb;
    std::time_t scheduledTime;
    bool cancelled;
    bool isRepeated;
    std::chrono::duration<int> repeatInterval;
    std::vector<std::weak_ptr<std::shared_ptr<Job>>> prerequisites;
};
} //namespace scheduler
} //namespace alica
