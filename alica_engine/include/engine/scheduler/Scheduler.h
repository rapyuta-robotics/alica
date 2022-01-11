#pragma once

#include "engine/IAlicaTimer.h"
#include "engine/scheduler/JobQueue.h"

#include <atomic>
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
            , _schedulerThreadFinished(false)
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

        // TODO: Remove _schedulerThreadFinished
        while(!_schedulerThreadFinished) {
            std::cerr << "waiting for scheduler thread to finish" << std::endl;
            _cv.notify_one();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        _thread.join();

        _repeatableJobs.clear();
    }

    void cancelJob(JobId jobId)
    {
        // Should be called by the scheduler thread
        if (!_repeatableJobs.erase(jobId)) {
            _jobQueue.erase({jobId, nullptr, std::nullopt}, [](const Job& job, const Job& otherJob) { return std::get<0>(job) == std::get<0>(otherJob); });
        }
    }

private:
    void run()
    {
        while (_running) {
            std::cerr << "still running" << std::endl;
            auto job = _jobQueue.pop();
            if (!job) {
                std::cerr << "try to get mutex" << std::endl;
                std::unique_lock<std::mutex> lock(_mutex);
                std::cerr << "got mutex" << std::endl;
                std::cerr << "start wait" << std::endl;
                _cv.wait(lock, [this, &job]() {
                    std::cerr << "check wait" << std::endl;
                    if (!_running) {
                        return true;
                    }
                    job = _jobQueue.pop();
                    return job.has_value();
                });
                std::cerr << "finished wait" << std::endl;
            }
            if (!job) {
                _schedulerThreadFinished = true;
                return;
            }
            auto&& [jobId, jobCb, repeatInterval] = std::move(*job);
            if (repeatInterval) {
                _repeatableJobs[jobId] = _timerFactory.createTimer(std::move(jobCb), std::move(*repeatInterval));
            } else {
                jobCb();
            }
        }
        std::cerr << "left while loop" << std::endl;
        // Execute all pending non repeatable jobs i.e. all init's & terminate's
        while (true) {
            std::cerr << "entered second loop" << std::endl;
            auto job = _jobQueue.pop();
            if (!job) {
                std::cerr << "did not receive job" << std::endl;
                _schedulerThreadFinished = true;
                return;
            }
            std::cerr << "received job" << std::endl;
            auto&& [jobId, jobCb, repeatInterval] = std::move(*job);
            (void) jobId;
            if (!repeatInterval) {
                jobCb();
            }
        }
        _schedulerThreadFinished = true;
        std::cerr << "finished run" << std::endl;
    }

    std::atomic<bool> _running;
    std::atomic<bool> _schedulerThreadFinished;
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
