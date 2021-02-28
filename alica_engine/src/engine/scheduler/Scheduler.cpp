#include "engine/scheduler/Scheduler.h"
#include "engine/AlicaEngine.h"

#include <algorithm>

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(int numberOfThreads, const alica::AlicaEngine* ae) : _ae(ae), _running(true)
{
    for (int i = 0; i < numberOfThreads; i++) {
        _workers.push_back(new std::thread(&Scheduler::workerFunction, this));
    }
}

Scheduler::~Scheduler()
{
    _running = false;
    _jobQueue.clear();

    for (auto worker : _workers) {
        _workerCV.notify_all();
        worker->join();
        delete worker;
    }
}

void Scheduler::schedule(std::shared_ptr<Job> job)
{
    if (job->scheduledTime.inNanoseconds() == 0) {
        job->scheduledTime = _ae->getAlicaClock().now();
    }

    if (job->inExecution || _jobQueue.isScheduled(job)) {
        return;
    }

    // schedule missing prerequisites
    for (auto prerequisite : job->prerequisites) {
        if (auto prerequisiteSharedPtr = prerequisite.lock())  {
            schedule(prerequisiteSharedPtr);
        }
    }

    _jobQueue.insert(job);
    _workerCV.notify_one();
}

void Scheduler::workerFunction()
{
    while(_running.load()) {
        Job *job;
        std::shared_ptr<Job> jobSharedPtr;
        bool executeJob = true;

        {
            std::unique_lock<std::mutex>lock(_workerMtx);
            // wait when queue is empty. Do no wait when queue has jobs or the schedulers destructor is called.
            _workerCV.wait(lock, [this]{return !_jobQueue.isEmpty() || !_running.load();});

            if (_jobQueue.isEmpty() || !_running.load()) {
                continue;
            }

            jobSharedPtr = _jobQueue.popNext();
            jobSharedPtr->inExecution = true;
            job = jobSharedPtr.get();

            for (std::weak_ptr<Job> j : job->prerequisites) {
                if (j.lock()) {
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
                    schedule(jobSharedPtr);
                }
            } catch (std::bad_function_call& e) {
                std::cerr << "ERROR: Bad function call\n";
            }
        } else if (job->cancelled) {
            std::cerr << "job cancelled" << std::endl;
        } else {
            jobSharedPtr->inExecution = false;
            schedule(jobSharedPtr);
        }

    }
}
} //namespace scheduler
} //namespace alica
