#include "engine/scheduler/JobQueue.h"

namespace alica {
namespace scheduler {
    void JobQueue::insert(std::shared_ptr<Job> job)
    {
        {
            std::unique_lock<std::mutex> lock(_mtx);
            _queue.push_back(std::move(job));
            std::sort(_queue.begin(), _queue.end());
        }
    }

    bool JobQueue::isScheduled(std::shared_ptr<Job> job)
    {
        {
            std::unique_lock<std::mutex> lock(_mtx);

            // check if job is already in queue
            auto it = std::find_if(_queue.begin(), _queue.end(), [&](std::shared_ptr<Job> const& queuedJob) {
                return *queuedJob == *job;
            });

            if (it != _queue.end()) {
                // do not add job to queue if already queued
                return true;
            }

            return false;
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
