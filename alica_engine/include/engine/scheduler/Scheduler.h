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
    Scheduler(const alica::AlicaEngine* ae, const YAML::Node& config);
    ~Scheduler();
    void schedule(std::shared_ptr<Job> job, bool notify = true);
    void terminate();
    int getNextJobID();
    void stopJob(int jobId);

private:
    const alica::AlicaEngine* _ae;
    JobQueue _jobQueue;
    JobQueue _repeatedJobQueue;
    std::mutex _workerMtx;
    std::mutex _workerNotifierMtx;
    std::condition_variable _workerCV;
    std::condition_variable _workerNotifierCV;
    std::vector<std::thread> _workers;
    std::atomic<bool> _running;
    std::atomic<int> _jobID;
    bool _notifierIsActive;

    std::unique_ptr<alica::TimerFactoryRos> _alicaTimerFactory;

    void workerFunction();
    void workerNotifier();
    void monitorJobs();

    // key: jobId, value: timer
    std::unordered_map<int, std::unique_ptr<alica::SyncStopTimerRos<ros::CallbackQueue>>> _timers;
};
} // namespace scheduler
} // namespace alica
