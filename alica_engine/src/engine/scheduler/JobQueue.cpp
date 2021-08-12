#include "engine/scheduler/JobQueue.h"

#include <alica_common_config/debug_output.h>

namespace alica
{
namespace scheduler
{
void JobQueue::insert(std::shared_ptr<Job>&& job)
{
    _queue.push_back(std::move(job));
}

std::shared_ptr<Job> JobQueue::getAvailableJob()
{
    if (_queue.empty()) {
        return nullptr;
    }

    auto it = _queue.begin();
    std::shared_ptr<Job> job = std::move(*it);
    _queue.erase(it);

    return std::move(job);
}

void JobQueue::clear()
{
    _queue.clear();
}

bool JobQueue::isEmpty() const
{
    return _queue.empty();
}
} // namespace scheduler
} // namespace alica
