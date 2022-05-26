#pragma once

#include <deque>
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
        _queue.push_back(std::move(value));
    }

    template <class... Args>
    void emplace(Args&&... args)
    {
        _queue.emplace_back(std::forward<Args>(args)...);
    }

    std::optional<T> pop()
    {
        if (_queue.empty()) {
            return std::nullopt;
        }
        std::optional<T> val{std::move(_queue.front())};
        _queue.pop_front();
        return val;
    }

    void clear()
    {
        _queue.clear();
    }

    template <class Compare>
    void erase(const T& key, Compare compare)
    {
        for (auto it = _queue.begin(); it != _queue.end(); ++it) {
            if (compare(key, (*it))) {
                _queue.erase(it);
                return;
            }
        }
    }

private:
    std::deque<T> _queue;
};

} // namespace scheduler
} // namespace alica
