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
    std::cerr << "adding a job..." << std::endl;
    for (auto job : job.get()->prerequisites) {
        if (auto jobSharedPtr = job.lock())  {
            queue.push_back(jobSharedPtr);
        }
    }
    queue.push_back(job);
}
} //namespace scheduler
} //namespace alica
