#pragma once

#include "engine/scheduler/Job.h"
#include <memory>
#include <mutex>

namespace alica {
namespace scheduler {

template <class T>
class FIFOQueue
{
public:
    void push(T&& value) {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.push_back(std::move(value));
    }
    
    std::optional<T> pop() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_queue.empty()) {
            return std::nullopt;
        }
        T val = _queue.front();
        _queue.pop();
    }
    
    void clear() {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.clear();
    }

private:
    std::mutex _mutex;
    std::deque<std::shared_ptr<Job>> _queue;
};

// TODO: use a lock free single producer/consumer queue

}
}

