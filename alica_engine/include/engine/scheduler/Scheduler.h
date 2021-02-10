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
        void add(std::shared_ptr<Job> job);
    private:
        std::vector<std::shared_ptr<Job>> queue;
    };
}
}
