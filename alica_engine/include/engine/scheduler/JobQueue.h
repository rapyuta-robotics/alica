#pragma once

#include "engine/scheduler/Job.h"
#include <memory>
#include <mutex>

namespace alica {
namespace scheduler {
    class JobQueue
    {
    public:
        void insert(std::shared_ptr<Job>&& job);
        void clear();
        std::shared_ptr<Job> getAvailableJob(alica::AlicaTime time);
        std::vector<std::weak_ptr<Job>> getPrerequisites(int id);
        void detectDelayedJobs(alica::AlicaTime time);
        bool isEmpty() const;
    private:
        std::vector<std::shared_ptr<Job>> _queue;
    };
}
}

