#pragma once

#include "engine/AlicaTimer.h"
#include "engine/scheduler/Job.h"
#include "engine/scheduler/JobQueue.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

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
    void stopJob(int jobId);

private:
    alica::AlicaClock _clock;
    JobQueue _jobQueue;
    std::mutex _workerMtx;
    std::condition_variable _workerCV;
    std::vector<std::thread> _workers;
    std::atomic<bool> _running;
    std::atomic<int> _jobID;
    bool _notifierIsActive;

    std::unique_ptr<alica::TimerFactoryRos> _alicaTimerFactory;

    // key: jobId, value: timer
    std::unordered_map<int, alica::SyncStopTimerRos<ros::CallbackQueue>> _timers;

    void workerFunction();
};
} // namespace scheduler
} // namespace alica
