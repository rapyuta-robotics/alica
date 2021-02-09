#include "engine/scheduler/Scheduler.h"

namespace alica
{

namespace scheduler
{

void Scheduler::add(std::weak_ptr<Job> job)
{
    std::cerr << "adding a job..." << std::endl;
//    AlicaClock clock;
//    auto sharedPtr = job.lock();
//    if (sharedPtr) {
//        sharedPtr.get()->scheduledTime = clock.now();
//        queue.push_back(job);
//    }
}
} //namespace scheduler
} //namespace alica
