#include "engine/scheduler/JobQueue.h"

namespace alica
{
namespace scheduler
{
void JobQueue::insert(std::shared_ptr<Job>&& job)
{
    _queue.insert(std::upper_bound(_queue.begin(), _queue.end(), job), std::move(job));
}

std::shared_ptr<Job> JobQueue::popNext()
{
    if (_queue.empty()) {
        return nullptr;
    }

    std::shared_ptr<Job> job = _queue.front();
    _queue.erase(_queue.begin());
    return job;
}

std::shared_ptr<Job> JobQueue::getAvailableJob(alica::AlicaTime time)
{
    if (_queue.empty()) {
        return nullptr;
    }

    auto it = _queue.begin();
    while (it != _queue.end()) {
        std::cerr << "jobid: " << (*it)->id << " pr: ";
        std::cerr << (*it)->prerequisites.size() << " finished: " << allPrerequisitesFinished(*it);
        std::cerr << " scheduledTime: " << (*it)->scheduledTime << " target: " << time << std::endl;
        if ((*it)->cancelled) {
            it = _queue.erase(it);
            continue;
        }

        if (allPrerequisitesFinished(*it) && (*it)->scheduledTime <= time) {
            std::shared_ptr<Job> job = std::move(*it);
            _queue.erase(it);
            return job;
        }
    }
    return nullptr;
}

bool JobQueue::allPrerequisitesFinished(std::shared_ptr<Job> job)
{
    for (std::weak_ptr<Job> prerequisite : job->prerequisites) {
        if (prerequisite.lock()) {
            return false;
        }
    }
    return true;
}

void JobQueue::clear()
{
    _queue.clear();
}
} // namespace scheduler
} // namespace alica
