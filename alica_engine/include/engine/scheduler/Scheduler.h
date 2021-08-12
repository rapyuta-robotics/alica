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
    Scheduler(const YAML::Node& config);
    ~Scheduler();
    int schedule(std::shared_ptr<Job>&& job, std::unique_ptr<alica::AlicaTime> value = nullptr);
    void terminate();
    void stopJob(int jobId);

private:
    JobQueue _jobQueue;
    std::mutex _workerMtx;
    std::condition_variable _workerCV;
    std::vector<std::thread> _workers;
    std::atomic<bool> _running;
    std::atomic<int> _jobId;
    bool _notifierIsActive;

    std::unique_ptr<alica::TimerFactoryRos> _alicaTimerFactory;

    // key: jobId, value: timer
    std::unordered_map<int, alica::SyncStopTimerRos<ros::CallbackQueue>> _timers;

    void workerFunction();
    int getNextJobId();
};
} // namespace scheduler
} // namespace alica
