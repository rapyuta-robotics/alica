#pragma once

#include "engine/scheduler/Job.h"
#include <memory>
#include <mutex>

namespace alica {
namespace scheduler {
    class JobQueue
    {
    public:
        void insert(std::shared_ptr<Job> job);
        bool isScheduled(std::shared_ptr<Job> job);
        bool isEmpty();
        std::shared_ptr<Job> popNext();
        void clear();

    private:
        std::vector<std::shared_ptr<Job>> _queue;
        std::mutex _mtx;
    };
}
}

