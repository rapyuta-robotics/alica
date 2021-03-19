#include "engine/scheduler/Scheduler.h"
#include "engine/AlicaEngine.h"

#include <algorithm>

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(int numberOfThreads, const alica::AlicaEngine* ae)
        : _ae(ae)
        , _running(true)
        , _jobID(0)
{
    for (int i = 0; i < numberOfThreads; i++) {
        _workers.emplace_back(std::thread(&Scheduler::workerFunction, this));
    }
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
    _jobQueue.clear();

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

    _jobQueue.insert(std::move(job));

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
            // wait when queue is empty. Do no wait when queue has jobs or the schedulers destructor is called.
            _workerCV.wait(lock, [this] { return !_jobQueue.isEmpty() || !_running.load(); });

            if (_jobQueue.isEmpty() || !_running.load()) {
                continue;
            }

            job = _jobQueue.popNext();

            if (job->cancelled) {
                /*
                 * If job is cancelled:
                 *  - Dont execute job.
                 *  - Dont schedule job.
                 */
                job.reset();
                _workerCV.notify_one();
                continue;
            }


            for (std::weak_ptr<Job> prerequisite : job->prerequisites) {
                if (prerequisite.lock()) {
                    // Prerequisite jobs not finished, do not execute job
                    executeJob = false;
                    break;
                }
            }
        }

        if (executeJob) {
            try {
                job->cb();
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
