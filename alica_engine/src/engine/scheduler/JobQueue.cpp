#include "engine/scheduler/JobQueue.h"

#include <alica_common_config/debug_output.h>

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
    if (_queue.empty()) {
        return nullptr;
    }

    _lowestScheduledTime = _queue[0]->scheduledTime;

    for (auto it = _queue.begin(); it != _queue.end(); it++) {
        if ((*it)->cancelled && !((*it)->inProgress)) {
            it = _queue.erase(it);

            if (it == _queue.end()) {
                return nullptr;
            }
            continue;
        }

        if ((*it)->isPrerequisiteFree() && (*it)->scheduledTime <= time && !((*it)->inProgress)) {
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

void JobQueue::detectDelayedJobs(alica::AlicaTime time)
{
    for (std::shared_ptr job : _queue) {
        if (job->isRepeated && job->scheduledTime + job->repeatInterval < time) {
            ALICA_ERROR_MSG("Repeating job with id " << job->id << " is delayed");
        }
    }
}

bool JobQueue::isEmpty() const
{
    return _queue.empty();
}

AlicaTime JobQueue::getLowestScheduledTime() const
{
    return _lowestScheduledTime;
}
} // namespace scheduler
} // namespace alica
