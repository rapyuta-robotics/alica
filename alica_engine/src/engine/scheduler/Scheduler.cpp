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
    {
        std::unique_lock<std::mutex> lock(_mtx);
        _queue.clear();
    }

    for (auto worker : _workers) {
        _condition.notify_all();
        worker->join();
        delete worker;
    }
}

void Scheduler::add(std::shared_ptr<Job> job)
{
    if (job->scheduledTime.inNanoseconds() == 0) {
        job->scheduledTime = _ae->getAlicaClock().now();
    }

    if (job->inExecution) {
        return;
    }

    {
        std::unique_lock<std::mutex> lock(_mtx);

        // check if job is already in queue
        auto it = std::find_if(_queue.begin(), _queue.end(), [&](std::shared_ptr<Job> const& queuedJob) {
            return *queuedJob == *(job.get());
        });

        if (it != _queue.end()) {
            // do not add job to queue if already queued
            std::cerr << "Scheduler: job already in queue" << std::endl;
            return;
        }
    }

    for (auto job : job.get()->prerequisites) {
        if (auto jobSharedPtr = job.lock())  {
            add(jobSharedPtr);
        }
    }

    {
        std::unique_lock<std::mutex> lock(_mtx);
        _queue.push_back(job);
        std::sort(_queue.begin(), _queue.end());
    }

    _condition.notify_one();
}

void Scheduler::workerFunction()
{
    while(_running.load()) {
        Job *job;
        std::shared_ptr<Job> jobSharedPtr;
        bool executeJob = true;

        {
            std::unique_lock<std::mutex> lock(_mtx);
            // wait when queue is empty. Do no wait when queue has jobs or the schedulers destructor is called.
            _condition.wait(lock, [this]{return !_queue.empty() || !_running.load();});

            if (_queue.empty() || !_running.load()) {
                continue;
            }

            jobSharedPtr = _queue.front();
            jobSharedPtr->inExecution = true;
            job = jobSharedPtr.get();
            _queue.erase(_queue.begin());

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
                    add(jobSharedPtr);
                }
            } catch (std::bad_function_call& e) {
                std::cerr << "ERROR: Bad function call\n";
            }
        } else if (job->cancelled) {
            std::cerr << "job cancelled" << std::endl;
        } else {
            jobSharedPtr->inExecution = false;
            add(jobSharedPtr);
        }

    }
}
} //namespace scheduler
} //namespace alica
