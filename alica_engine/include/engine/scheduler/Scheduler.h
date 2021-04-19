#pragma once

#include "engine/scheduler/Job.h"
#include "engine/scheduler/JobQueue.h"
#include "engine/AlicaClock.h"

#include <alica_common_config/debug_output.h>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>
#include <yaml-cpp/yaml.h>

namespace alica
{
    class AlicaEngine;

namespace scheduler
{
    class Scheduler
    {
    public:
        Scheduler(const alica::AlicaClock& clock, const YAML::Node& config);
        ~Scheduler();
        void schedule(std::shared_ptr<Job>&& job, bool notify = true);
        void terminate();
        int getNextJobID();
        //Used by unit tests only
        std::vector<std::weak_ptr<Job>> getPrerequisites(int id);
    private:
        alica::AlicaClock _clock;
        JobQueue _jobQueue;
        JobQueue _repeatedJobQueue;
        std::mutex _workerMtx;
        std::condition_variable _workerCV;
        std::vector<std::thread> _workers;
        std::atomic<bool> _running;
        std::atomic<int> _jobID;

        void workerFunction();
    };
}
}
