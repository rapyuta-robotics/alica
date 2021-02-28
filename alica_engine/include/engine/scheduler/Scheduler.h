#pragma once

#include "engine/scheduler/Job.h"
#include "engine/scheduler/JobQueue.h"
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
        JobQueue _jobQueue;
        std::mutex _workerMtx;
        std::condition_variable _workerCV;
        std::vector<std::thread*> _workers;
        std::atomic<bool> _running;

        void workerFunction();
    };
}
}
