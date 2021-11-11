#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTrace.h"

#include <atomic>
#include <memory>
#include <string>

namespace alica
{
namespace test
{
class TestContext;
}
class Configuration;
class AlicaEngine;
class RunningPlan;
class ThreadSafePlanInterface;

/**
 * The base class for BasicBehaviour and BasicPlan
 */
class RunnableObject
{
public:
    RunnableObject(const std::string& name = "");
    virtual ~RunnableObject() = default;
    void setEngine(AlicaEngine* engine);
    void setConfiguration(const Configuration* conf);
    void setName(const std::string& name);
    AlicaTime getInterval() const;
    void setInterval(int32_t msInterval);
    void disableTracing();
    void stop();
    void start(RunningPlan* rp);

    // This is not thread safe. Should only be called by the scheduler thread. TODO: make this private
    std::optional<std::string> getTraceContext() const;

protected:
    using Counter = uint64_t;

    static constexpr int DEFAULT_MS_INTERVAL = 100;

    enum class Flags : uint8_t
    {
        INIT_EXECUTED = 1u,
        TRACING_ENABLED = 1u << 1,
        RUN_TRACED = 1u << 2
    };

    AlicaEngine* _engine;
    const Configuration* _configuration;
    std::unique_ptr<IAlicaTrace> _trace;
    std::string _name;
    AlicaTime _msInterval;
    uint8_t _flags;
    std::atomic<Counter> _execState;          // Tracks the actual executate state of the behaviour by the scheduler thread
    std::atomic<Counter> _signalState;        // Tracks the signal state from the alica main engine thread i.e. tracks start() & stop() calls
    std::atomic<RunningPlan*> _signalContext; // The running plan context when start() is called
    std::atomic<RunningPlan*> _execContext;   // The running plan context under which the behaviour is executing
    int64_t _activeRunJobId;

    virtual void doInit() = 0;
    virtual void doTerminate() = 0;

    std::optional<IAlicaTrace*> getTrace() const;
    void sendLogMessage(int level, const std::string& message) const;
    void setFlags(Flags flags) { _flags |= static_cast<uint8_t>(flags); }
    void clearFlags(Flags flags) { _flags &= ~static_cast<uint8_t>(flags); }
    bool areFlagsSet(Flags flags) { return (static_cast<uint8_t>(flags) & _flags) == static_cast<uint8_t>(flags); }
    bool isExecutingInContext() const;
    // If the counter is even then it indicates the behaviour is active i.e it is started, but not stopped yet
    static bool isActive(Counter cnt);
    ThreadSafePlanInterface getPlanContext() const;
    void setTerminatedState();
    void traceTermination();
    void initTrace();
    void traceRun();
    void traceInit(const std::string& type);
};
} /* namespace alica */
