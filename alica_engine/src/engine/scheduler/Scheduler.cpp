#include "engine/scheduler/Scheduler.h"
#include "engine/AlicaEngine.h"

#include <algorithm>

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(const alica::AlicaEngine* ae, const YAML::Node& config)
        : _ae(ae)
        , _jobID(0)
        , _notifierIsActive(false)
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
        _workerNotifierCV.notify_one();
        worker.join();
    }
    _workers.clear();
}

void Scheduler::schedule(std::shared_ptr<Job> job, bool notify)
{
    if (job->scheduledTime.inNanoseconds() == 0) {
        job->scheduledTime = _ae->getAlicaClock().now();
    }

    {
        std::unique_lock<std::mutex> lock(_workerMtx);
        if (job->isRepeated) {
            auto timer = _alicaTimerFactory->createTimer(job->cb, job->repeatInterval);
            _timers.emplace(job->id, std::move(timer));
        } else {
            _jobQueue.insert(std::move(job));
        }
    }

    if (notify) {
        _workerCV.notify_one();
    }
}

void Scheduler::stopJob(int jobId)
{
    auto it = _timers.find(jobId);

    if (it != _timers.end()) {
        it->second->stop();
        it->second.reset();
        _timers.erase(it);
    }
}

int Scheduler::getNextJobID()
{
    return ++_jobID;
}

std::vector<std::weak_ptr<Job>> Scheduler::getPrerequisites(int id)
{
    return _jobQueue.getPrerequisites(id);
}

void Scheduler::workerFunction()
{
    while (_running.load()) {
        std::shared_ptr<Job> job;
        {
            std::unique_lock<std::mutex> lock(_workerMtx);
            // wait when no executable job is available. Do no wait when queue has executable jobs or the scheduler has been terminated.
            _workerCV.wait(lock, [this, &job] {
                job = _jobQueue.getAvailableJob(_ae->getAlicaClock().now());
                return !_running.load() || job;
            });

            if (!_running.load()) {
                continue;
            }
        }

        try {
            std::cerr << "execute job: " << job->id << std::endl;
            job->cb();
            std::cerr << "finished job: " << job->id << std::endl;
        } catch (const std::bad_function_call& e) {
            ALICA_ERROR_MSG("Bad function call");
        }
        job.reset();
        _workerCV.notify_one();
    }
}
} // namespace scheduler
} // namespace alica
