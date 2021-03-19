#include "engine/scheduler/JobQueue.h"

namespace alica {
namespace scheduler {
    void JobQueue::insert(std::shared_ptr<Job>&& job)
    {
        {
            std::unique_lock<std::mutex> lock(_mtx);
            _queue.insert(std::upper_bound(_queue.begin(), _queue.end(), job), std::move(job));
        }
    }

    bool JobQueue::isEmpty()
    {
        {
            std::unique_lock<std::mutex> lock(_mtx);
            return _queue.empty();
        }
    }

    std::shared_ptr<Job> JobQueue::popNext()
    {
        if (isEmpty()) {
            return nullptr;
        }

        {
            std::unique_lock<std::mutex> lock(_mtx);
            std::shared_ptr<Job> job = _queue.front();
            _queue.erase(_queue.begin());
            return job;
        }
    }

    void JobQueue::clear()
    {
        {
            std::unique_lock<std::mutex> lock(_mtx);
            _queue.clear();
        }
    }
}
}
