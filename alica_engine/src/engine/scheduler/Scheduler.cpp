#include "engine/scheduler/Scheduler.h"

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(int numberOfThreads)
{
    for (int i = 0; i < numberOfThreads; i++) {
        _workers.push_back(new std::thread(&Scheduler::workerFunction, this));
    }
}

Scheduler::~Scheduler()
{
    running = false;
    {
        std::unique_lock<std::mutex> lock(mtx);
        queue.clear();
    }

    for (auto worker : _workers) {
        condition.notify_all();
        worker->join();
        delete worker;
    }
}

void Scheduler::add(std::shared_ptr<Job> job)
{
    // check if job is already in queue
    auto it = std::find_if(queue.begin(), queue.end(), [&](std::shared_ptr<Job> const& queuedJob) {
       return *queuedJob == *(job.get());
    });

    if (it == queue.end()) {
        // do not add job to queue if already queued
        return;
    }

    std::cerr << "adding a job..." << std::endl;
    for (auto job : job.get()->prerequisites) {
        if (auto jobSharedPtr = job.lock())  {
            add(jobSharedPtr);
        }
    }
    {
        std::unique_lock<std::mutex> lock(mtx);
        queue.push_back(job);
    }

    condition.notify_one();
}

void Scheduler::workerFunction()
{
    while(running) {
        Job *job;
        std::shared_ptr<Job> jobSharedPtr;
        bool executeJob = true;

        {
            std::unique_lock<std::mutex> lock(mtx);
            // wait when queue is empty. Do no wait when queue has jobs or the schedulers destructor is called.
            condition.wait(lock, [this]{return !queue.empty() || !running;});

            if (queue.empty() || !running) {
                continue;
            }

            jobSharedPtr = queue.front();
            job = jobSharedPtr.get();
            queue.erase(queue.begin());

            for (std::weak_ptr<Job> j : job->prerequisites) {
                if (j.lock()) {
                    // Prerequisite jobs not finished, do not execute job
                    executeJob = false;
                    break;
                }
            }
        }

        if (executeJob) {
            job->cb();
        } else {
            add(jobSharedPtr);
        }

    }
}
} //namespace scheduler
} //namespace alica
