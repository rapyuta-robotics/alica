#pragma once

#include "engine/AlicaTimer.h"
#include "engine/scheduler/JobQueue.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace alica
{
class AlicaEngine;

namespace scheduler
{

using JobId = int64_t;
using JobCb = std::function<void()>;

struct Job
{
    JobId id;
    std::function<void()> cb;
    std::optional<AlicaTime> repeatInterval;
};

template <template <class> class Queue>
class Scheduler
{
public:
    //    using JobId = int64_t;
    //    using JobCb = std::function<void()>;

private:
    //    struct Job
    //    {
    //        JobId id;
    //        std::function<void()> cb;
    //        std::optional<AlicaTime> repeatInterval;
    //    };

    using JobQueue = Queue<Job>;
    using Timer = alica::SyncStopTimerRos<ros::CallbackQueue>;

public:
    using JobTest = Job;
    Scheduler(const YAML::Node& config)
    {
        _running = true;
        int threadPoolSize = config["Alica"]["ThreadPoolSize"].as<int, int>(std::max(1u, std::thread::hardware_concurrency()));

        _alicaTimerFactory = std::make_unique<alica::TimerFactoryRos>(threadPoolSize);
        //        _workers.emplace_back(std::thread(&Scheduler::workerFunction, this));
        _worker = std::thread(&Scheduler::workerFunction, this);
    }

    ~Scheduler()
    {
        if (_running) {
            terminate();
        }
    }

    JobId schedule(JobCb jobCb, std::optional<AlicaTime> repeatInterval = std::nullopt)
    {
        Job job;
        JobId jobId = ++_nextJobId;
        job.id = jobId;
        job.cb = std::move(jobCb);
        job.repeatInterval = repeatInterval;
        _jobQueue.push(std::move(job));
        _cv.notify_one();
        return jobId;
    }

    void terminate()
    {
        _running = false;
        _jobQueue.clear();

        _cv.notify_all();
        _worker.join();

        auto it = _timers.begin();
        while (it != _timers.end()) {
            it->second->stop();
            it = _timers.erase(it);
        }
    }
    void stopJob(JobId jobId) { _timers.erase(jobId); }

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
    std::thread _worker;
    mutable std::mutex _mutex;
    std::condition_variable _cv;
    std::atomic<bool> _running;
    std::unordered_map<JobId, std::unique_ptr<Timer>> _timers;
    alica::TimerFactoryRos _alicaTimerFactory;
};
using JobScheduler = Scheduler<typename FIFOQueue<Job>>;
} // namespace scheduler
} // namespace alica
