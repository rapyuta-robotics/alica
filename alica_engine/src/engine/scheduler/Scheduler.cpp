#include "engine/scheduler/Scheduler.h"

#include <algorithm>
#include <alica_common_config/debug_output.h>

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(const YAML::Node& config)
        : _jobId(0)
{
    _running = true;
    int threadPoolSize = config["Alica"]["ThreadPoolSize"].as<int, int>(std::max(1u, std::thread::hardware_concurrency()));

    _alicaTimerFactory = std::make_unique<alica::TimerFactoryRos>(threadPoolSize);
    _workers.emplace_back(std::thread(&Scheduler::workerFunction, this));
}

Scheduler::~Scheduler()
{
    if (_running.load()) {
        terminate();
    }
}

void Scheduler::terminate()
{
    _running = false;
    {
        std::unique_lock<std::mutex> lock(_workerMtx);
        _jobQueue.clear();
    }

    for (std::thread& worker : _workers) {
        _workerCV.notify_all();
        worker.join();
    }
    _workers.clear();

    auto it = _timers.begin();
    while (it != _timers.end()) {
        it->second.stop();
        it = _timers.erase(it);
    }
}

int Scheduler::schedule(std::shared_ptr<Job>&& job, std::unique_ptr<alica::AlicaTime> repeatInterval)
{
    int jobId = -1;
    {
        std::unique_lock<std::mutex> lock(_workerMtx);
        if (repeatInterval) {
            jobId = getNextJobId();
            _timers.emplace(jobId, std::move(_alicaTimerFactory->createTimer(job->cb, *repeatInterval)));
            _timers.at(jobId).start();
        } else {
            _jobQueue.insert(std::move(job));
        }
    }

    _workerCV.notify_one();
    return jobId;
}

void Scheduler::stopJob(int jobId)
{
    auto it = _timers.find(jobId);

    if (it != _timers.end()) {
        it->second.stop();
        _timers.erase(it);
    }
}

int Scheduler::getNextJobId()
{
    return ++_jobId;
}

void Scheduler::workerFunction()
{
    while (_running.load()) {
        std::shared_ptr<Job> job;
        {
            std::unique_lock<std::mutex> lock(_workerMtx);
            // wait when no executable job is available. Do no wait when queue has executable jobs or the scheduler has been terminated.
            _workerCV.wait(lock, [this, &job] {
                job = std::move(_jobQueue.getAvailableJob());
                return !_running.load() || job;
            });

            if (!_running.load()) {
                continue;
            }
        }

        try {
            job->cb();
        } catch (const std::bad_function_call& e) {
            ALICA_ERROR_MSG("Bad function call");
        }
        job.reset();
        _workerCV.notify_one();
    }
}
} // namespace scheduler
} // namespace alica
