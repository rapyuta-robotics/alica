#pragma once

#include "engine/AlicaClock.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <vector>
#include <memory>
#include <functional>

namespace alica {
namespace scheduler
{
struct Job
{
    Job(std::function<void()> cb, std::vector<std::weak_ptr<Job>> prerequisites)
        : cb(cb)
        , prerequisites(prerequisites)
        , cancelled(false)
        , isRepeated(false)
        , inExecution(false)
    { }

    std::function<void()> cb;
    AlicaTime scheduledTime;
    bool cancelled;
    bool isRepeated;
    bool inExecution;
    AlicaTime repeatInterval;
    std::vector<std::weak_ptr<Job>> prerequisites;

    bool operator==(const Job& r) const
    {
        return scheduledTime == r.scheduledTime;
    }

    bool isPrerequisite(const Job& other) const
    {
        for (std::weak_ptr<Job> jobWeakPtr : prerequisites) {
            std::shared_ptr<Job> job = jobWeakPtr.lock();
            if (job && *(job) == other) {
                return true;
            }
        }
        return false;
    }

    bool operator<(const Job& other) const
    {
        return isPrerequisite(other)
                || (!other.isPrerequisite(*this) && scheduledTime < other.scheduledTime);
    }

};
} //namespace scheduler
} //namespace alica
