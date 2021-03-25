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

std::vector<std::weak_ptr<Job>> JobQueue::getPrerequisites(int id) {
    for (std::shared_ptr job : _queue) {
        if (job->id == id) {
            return job->prerequisites;
        }
    }
    return std::vector<std::weak_ptr<Job>>();
}

std::shared_ptr<Job> JobQueue::getAvailableJob(alica::AlicaTime time)
{
    if (_queue.empty()) {
        return nullptr;
    }

    auto it = _queue.begin();
    while (it != _queue.end()) {
        if ((*it)->cancelled) {
            it = _queue.erase(it);
            continue;
        }

        if ((*it)->isPrerequisiteFree() && (*it)->scheduledTime <= time) {
            std::shared_ptr<Job> job = std::move(*it);
            _queue.erase(it);
            return job;
        }
    }
    return nullptr;
}

void JobQueue::clear()
{
    _queue.clear();
}
} // namespace scheduler
} // namespace alica
