#pragma once
#include "engine/IExecutor.h"
#include <atomic>
#include <memory>
#include <thread>

namespace alica
{

class AsyncExecutor : public IExecutor
{

public:
    AsyncExecutor(){};
    ~AsyncExecutor() override { stop(); };

    void start() override;
    void stop() override;
    void run() override;

private:
    std::unique_ptr<std::thread> _thread;
    std::atomic<bool> _running{false};
};

class DrivenExecutor : public IExecutor
{
public:
    DrivenExecutor(){};
    ~DrivenExecutor() override = default;

    void start() override{};
    void stop() override{};
    void run() override;
};

} // namespace alica
