#pragma once

#include "engine/IAlicaTimer.h"
#include "engine/scheduler/JobQueue.h"

#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <atomic>
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
    using Job = std::tuple<JobId, JobCb, std::optional<AlicaTime>>;

private:
    using JobQueue = Queue<Job>;

public:
    Scheduler(IAlicaTimerFactory& timerFactory)
            : _running(false)
            , _jobQueue()
            , _mutex()
            , _cv()
            , _thread()
            , _nextJobId(1)
            , _repeatableJobs()
            , _timerFactory(timerFactory)
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
        JobId jobId = _nextJobId++;
        _jobQueue.emplace(jobId, std::move(jobCb), std::move(repeatInterval));
        _cv.notify_one();
        return jobId;
    }

    void init()
    {
        _running = true;
        _thread = std::thread(&Scheduler::run, this);
    }

    void terminate()
    {
        _running = false;

        _cv.notify_one();
        _thread.join();

        _repeatableJobs.clear();
    }

    void cancelJob(JobId jobId)
    {
        // Should be called by the scheduler thread
        if (!_repeatableJobs.erase(jobId)) {
            _jobQueue.erase({jobId, nullptr, std::nullopt},
                            [](const Job& job, const Job& otherJob) {
                        return std::get<0>(job) == std::get<0>(otherJob);
                    });
        }
    }

private:
    void run()
    {
        while (_running) {
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
            if (!job) {
                return;
            }
            auto&& [jobId, jobCb, repeatInterval] = std::move(*job);
            if (repeatInterval) {
                _repeatableJobs[jobId] = _timerFactory.createTimer(std::move(jobCb), std::move(*repeatInterval));
            } else {
                jobCb();
            }
        }
        // Execute all pending non repeatable jobs i.e. all init's & terminate's
        while (true) {
            auto job = _jobQueue.pop();
            if (!job) {
                return;
            }
            auto&& [jobId, jobCb, repeatInterval] = std::move(*job);
            if (!repeatInterval) {
                jobCb();
            }
        }
    }

    std::atomic<bool> _running;
    JobQueue _jobQueue;
    mutable std::mutex _mutex;
    std::condition_variable _cv;
    std::thread _thread;
    std::atomic<JobId> _nextJobId;
    std::unordered_map<JobId, std::unique_ptr<IAlicaTimer>> _repeatableJobs;
    IAlicaTimerFactory& _timerFactory;
};

using JobScheduler = Scheduler<FIFOQueue>;

} // namespace scheduler
} // namespace alica
