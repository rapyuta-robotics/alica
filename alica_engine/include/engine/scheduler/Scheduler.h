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
        void add(std::weak_ptr<Job> job)
        {
            AlicaClock clock;
            auto sharedPtr = job.lock();
            if (sharedPtr) {
                sharedPtr.get()->scheduledTime = clock.now();
                queue.push_back(job);
            }

        };
    private:
        std::vector<std::weak_ptr<Job>> queue;
    };
}
}
