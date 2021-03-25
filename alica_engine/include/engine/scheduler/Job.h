#pragma once

#include "engine/AlicaClock.h"

#include <functional>
#include <memory>
#include <vector>

namespace alica
{
namespace scheduler
{
struct Job
{
    int id;
    std::function<void()> cb;
    AlicaTime scheduledTime;
    bool cancelled;
    bool isRepeated;
    AlicaTime repeatInterval;
    std::vector<std::weak_ptr<Job>> prerequisites;

    Job(int id, std::function<void()> cb, std::vector<std::weak_ptr<Job>> prerequisites)
            : cb(cb)
            , id(id)
            , prerequisites(prerequisites)
            , cancelled(false)
            , isRepeated(false)
    {}

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

    bool operator==(const Job& r) const { return id == r.id; }

    bool operator<(const Job& other) const {
        return scheduledTime < other.scheduledTime;
    }
};
} // namespace scheduler
} // namespace alica
