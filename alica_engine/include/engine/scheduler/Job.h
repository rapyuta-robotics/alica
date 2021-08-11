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
    bool isRepeated;
    AlicaTime repeatInterval;

    Job(int id, std::function<void()> cb)
            : cb(cb)
            , id(id)
            , isRepeated(false)
    {}

    bool operator==(const Job& r) const { return id == r.id; }
};
} // namespace scheduler
} // namespace alica
