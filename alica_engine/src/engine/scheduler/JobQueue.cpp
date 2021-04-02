#include "engine/scheduler/JobQueue.h"

namespace alica
{
namespace scheduler
{
void JobQueue::insert(std::shared_ptr<Job>&& job)
{
    _queue.insert(std::upper_bound(_queue.begin(), _queue.end(), job), std::move(job));
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
    _lowestScheduledTime = time;

    if (_queue.empty()) {
        return nullptr;
    }

    for (auto it = _queue.begin(); it != _queue.end(); it++) {
        if ((*it)->cancelled) {
            it = _queue.erase(it);
            continue;
        }

        if ((*it)->isPrerequisiteFree() && (*it)->scheduledTime <= time) {
            if ((*it)->isRepeated) {
                return (*it);
            }
            std::shared_ptr<Job> job = std::move(*it);
            _queue.erase(it);
            return std::move(job);
        }

        if ((*it)->isPrerequisiteFree() && !(*it)->inProgress) {
            _lowestScheduledTime = std::min((*it)->scheduledTime, _lowestScheduledTime);
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
