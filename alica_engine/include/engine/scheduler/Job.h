#pragma once

#include "engine/AlicaClock.h"

#include <iostream>
#include <chrono>
#include <ctime>
#include <vector>
#include <memory>
#include <functional>
#include <atomic>

namespace alica {
namespace scheduler
{
struct Job
{
    int id;
    std::function<void()> cb;
    AlicaTime scheduledTime;
    bool cancelled;
    bool isRepeated;
    bool inExecution;
    AlicaTime repeatInterval;
    std::vector<std::weak_ptr<Job>> prerequisites;

    Job(std::function<void()> cb, std::vector<std::weak_ptr<Job>> prerequisites)
        : cb(cb)
        , prerequisites(prerequisites)
        , cancelled(false)
        , isRepeated(false)
        , inExecution(false)
    {
        static std::atomic<int> sId{0};
        id = ++sId;
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

    bool operator==(const Job& r) const
    {
        return id == r.id;
    }

    bool operator<(const Job& other) const
    {
        return isPrerequisite(other)
                || (!other.isPrerequisite(*this) && scheduledTime < other.scheduledTime);
    }

};
} //namespace scheduler
} //namespace alica
