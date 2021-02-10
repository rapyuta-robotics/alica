#include "engine/scheduler/Scheduler.h"

namespace alica
{

namespace scheduler
{

Scheduler::Scheduler(int numberOfThreads)
{
    std::cerr << "Scheduler Threads: " << numberOfThreads << std::endl;
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
    queue.push_back(job);
}
} //namespace scheduler
} //namespace alica
