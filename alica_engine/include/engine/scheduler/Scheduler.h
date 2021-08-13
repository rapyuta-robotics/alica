#pragma once

#include "engine/AlicaTimer.h"
#include "engine/scheduler/JobQueue.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

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
    using JobQueue = Queue<std::tuple<JobId, JobCb, std::optional<AlicaTime>>>;
    using Timer = alica::TimerFactoryRos::TimerType;

public:
    Scheduler(const YAML::Node& config)
            : _running(true)
            , _jobQueue()
            , _mutex()
            , _cv()
            , _thread(&Scheduler::run, this)
            , _nextJobId(1)
            , _repeatableJobs()
            , _timerFactory(config["Alica"]["ThreadPoolSize"].as<int, int>(std::max(1u, std::thread::hardware_concurrency())))
    {
    }

    ~Scheduler()
    {
        if (_running) {
            terminate();
        }
    }

    JobId schedule(JobCb jobCb, std::optional<AlicaTime> repeatInterval = std::nullopt)
    {
        _jobQueue.emplace(_nextJobId++, std::move(jobCb), std::move(repeatInterval));
        _cv.notify_one();
        return _nextJobId;
    }

    void terminate()
    {
        _running = false;
        _jobQueue.clear();

        _cv.notify_one();
        _thread.join();

        _repeatableJobs.clear();
    }

    void stopJob(JobId jobId) { _repeatableJobs.erase(jobId); }

private:
    void run()
    {
        while(_running) {
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
            auto&& [jobId, jobCb, repeatInterval] = std::move(*job);
            if (repeatInterval) {
                _repeatableJobs[jobId] = _timerFactory.createTimer(std::move(jobCb), std::move(*repeatInterval));
                _repeatableJobs[jobId]->start();
            } else {
                jobCb();
            }
        }
    }

    std::atomic<bool> _running;
    JobQueue _jobQueue;
    mutable std::mutex _mutex;
    std::condition_variable _cv;
    std::thread _thread;
    JobId _nextJobId;
    std::unordered_map<JobId, std::unique_ptr<Timer>> _repeatableJobs;
    alica::TimerFactoryRos _timerFactory;
};

using JobScheduler = Scheduler<FIFOQueue>;

} // namespace scheduler
} // namespace alica
