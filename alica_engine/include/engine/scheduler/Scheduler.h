#pragma once

#include "engine/scheduler/Job.h"
#include <vector>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>

namespace alica
{
    class AlicaEngine;

namespace scheduler
{
    class Scheduler
    {
    public:
        Scheduler(int numberOfThreads, const alica::AlicaEngine* ae);
        ~Scheduler();
        void schedule(std::shared_ptr<Job> job);
    private:
        const alica::AlicaEngine* _ae;
        std::vector<std::shared_ptr<Job>> _queue;
        std::condition_variable _condition;
        std::vector<std::thread*> _workers;
        std::mutex _mtx;
        std::atomic<bool> _running;

        void workerFunction();
    };
}
}
