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
    std::function<void()> cb;
    AlicaTime repeatInterval;

    Job(std::function<void()> cb)
            : cb(cb)
    {}
};
} // namespace scheduler
} // namespace alica
