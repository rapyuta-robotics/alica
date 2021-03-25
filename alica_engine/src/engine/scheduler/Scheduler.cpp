#include "engine/scheduler/Scheduler.h"
#include "engine/AlicaEngine.h"

#include <algorithm>

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(const alica::AlicaEngine* ae)
        : _ae(ae)
        , _jobID(0)
{}

Scheduler::~Scheduler()
{
    if (_running.load()) {
        terminate();
    }
}

void Scheduler::init(int numberOfThreads)
{
    _running = true;
    for (int i = 0; i < numberOfThreads; i++) {
        _workers.emplace_back(std::thread(&Scheduler::workerFunction, this));
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
}

void Scheduler::schedule(std::shared_ptr<Job> job, bool notify)
{
    if (job->scheduledTime.inNanoseconds() == 0) {
        job->scheduledTime = _ae->getAlicaClock().now();
    }

    {
        std::unique_lock<std::mutex> lock(_workerMtx);
        if (job->isRepeated) {
            _repeatedJobQueue.insert(std::move(job));
        } else {
            _jobQueue.insert(std::move(job));
        }
    }

    if (notify) {
        _workerCV.notify_one();
    }
}

int Scheduler::getNextJobID()
{
    return ++_jobID;
}

std::shared_ptr<Job> Scheduler::popNext()
{
    return _jobQueue.popNext();
}

void Scheduler::workerFunction()
{
    while (_running.load()) {
        std::shared_ptr<Job> job;
        bool executeJob = true;

        {
            std::unique_lock<std::mutex> lock(_workerMtx);
            // wait when no executable job is available. Do no wait when queue has executable jobs or the scheduler has been terminated.
            _workerCV.wait(lock, [this, &job] { return (job = std::move(_jobQueue.getAvailableJob(_ae->getAlicaClock().now()))) || !_running.load(); });
            std::cerr << "returned shared ptr: " << job << std::endl;

            if (!_running.load()) {
                continue;
            }
        }

        if (executeJob) {
            try {
                std::cerr << "executing job with id " << job->id << std::endl;
                job->cb();
                std::cerr << "finished execution of job with id " << job->id << std::endl;
                if (job->isRepeated) {
                    schedule(std::move(job));
                } else {
                    job.reset();
                    _workerCV.notify_one();
                }
            } catch (const std::bad_function_call& e) {
                ALICA_ERROR_MSG("Bad function call");
            }
        } else {
            schedule(std::move(job), false);
        }
    }
}
} // namespace scheduler
} // namespace alica
