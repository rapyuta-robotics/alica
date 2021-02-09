#pragma once

#include "engine/scheduler/Job.h"
#include <vector>

namespace alica
{
namespace scheduler
{
    class Scheduler
    {
    public:
        void add(std::weak_ptr<Job> job);
    private:
        std::vector<std::weak_ptr<Job>> queue;
    };
}
}
