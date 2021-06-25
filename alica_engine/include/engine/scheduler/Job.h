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
    bool inProgress;
    AlicaTime repeatInterval;
    std::vector<std::weak_ptr<Job>> prerequisites;
    int prerequisiteIndex;

    Job(int id, std::function<void()> cb, std::vector<std::weak_ptr<Job>> prerequisites)
            : cb(cb)
            , id(id)
            , prerequisites(prerequisites)
            , cancelled(false)
            , isRepeated(false)
            , inProgress(false)
            , prerequisiteIndex(0)
    {}

    bool isPrerequisiteFree()
    {
        for (uint i = prerequisiteIndex; i < prerequisites.size(); i++) {
            if (prerequisites.at(i).lock()) {
                return false;
            }
            prerequisiteIndex++;
        }
        return true;
    }

    bool operator==(const Job& r) const { return id == r.id; }

    bool operator<(const Job& other) const { return scheduledTime < other.scheduledTime; }
};
} // namespace scheduler
} // namespace alica
