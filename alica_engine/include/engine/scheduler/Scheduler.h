#pragma once

#include "engine/AlicaTimer.h"
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

template <template <class> class Queue>
class Scheduler
{
public:
    using JobId = int64_t;
    using JobCb = std::function<void()>;
    
private:
    struct Job
    {
        JobId id;
        std::function<void()> cb;
        std::optional<AlicaTime> repeatInterval;
    };
    
    using JobQueue = Queue<Job>;
    using Timer = alica::SyncStopTimerRos<ros::CallbackQueue>>;

public:
    Scheduler(const YAML::Node& config);
    ~Scheduler();
    
    JobId schedule(JobCb jobCb, std::optional<AlicaTime> repeatInterval)
    {
        Job job;
        job.id = ++_nextJobId;
        job.cb = stgd::move(jobCb);
        job.repeatInterval = repeatInterval;
        _jobQueue.push(std::move(job));
        _cv.notify_one();
    }
    
    // int schedule(std::shared_ptr<Job>&& job, std::unique_ptr<alica::AlicaTime> value = nullptr);
    void terminate();
    void stopJob(JobId jobId)
    {
        _timers.erase(jobId);
    }

private:
    void run()
    {
        auto job = _jobQueue.pop();
        if (!job) {
            std::unique_lock<std::mutex> lock(_mutex);
            _cv.wait(lock, [this, &job]() {
                if (!_running) {
                    return true;
                }
                job = _jobQueue.pop();
                return job.has_value();
            });
        }
        if (!_running) {
            return;
        }
        if (job.repeatInterval) {
            _timers[job.id] = _alicaTimerFactory.createTimer(std::move(job.cb), job.repeatInterval);
        } else {
            job.cb();
        }
    }

    JobQueue _jobQueue;
    JobId _nextJobId;
    mutable std::mutex _mutex;
    std::condition_variable _cv;
    std::atomic<bool> _running;
    
    // std::mutex _timerMtx;
    // std::mutex _workerMtx;
    // std::condition_variable _workerCV;
    // std::vector<std::thread> _workers;
    // std::atomic<bool> _running;
    // std::atomic<int> _jobId;
    // bool _notifierIsActive;


    // key: jobId, value: timer
    std::unordered_map<JobId, std::unique_ptr<Timer> _timers;
    alica::TimerFactoryRos _alicaTimerFactory;
};
} // namespace scheduler
} // namespace alica
