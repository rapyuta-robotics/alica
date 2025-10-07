#pragma once
#include <functional>

namespace alica
{
class IExecutor
{
public:
    virtual ~IExecutor() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void run() = 0; // only for driven executor
    void attachRunCb(std::function<void()>&& runCb);

protected:
    std::function<void()> _run_cb = nullptr;
};
} // namespace alica
