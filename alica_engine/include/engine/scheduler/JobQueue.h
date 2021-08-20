#pragma once

#include <deque>
#include <mutex>
#include <optional>

namespace alica
{
namespace scheduler
{

template <class T>
class FIFOQueue
{
public:
    void push(T&& value)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.push_back(std::move(value));
    }

    template <class... Args>
    void emplace(Args&&... args)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.emplace_back(std::forward<Args>(args)...);
    }

    std::optional<T> pop()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_queue.empty()) {
            return std::nullopt;
        }
        std::optional<T> val{std::move(_queue.front())};
        _queue.pop_front();
        return val;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _queue.clear();
    }

    template <class Compare>
    void erase(const T& key, Compare compare)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        for (auto it = _queue.begin(); it != _queue.end(); ++it) {
            if (compare(key, (*it))) {
                _queue.erase(it);
                return;
            }
        }
    }

private:
    std::mutex _mutex;
    std::deque<T> _queue;
};

// TODO: use a lock free single producer/consumer queue

} // namespace scheduler
} // namespace alica
