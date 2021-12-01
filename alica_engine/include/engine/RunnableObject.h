#pragma once

#include "engine/AlicaClock.h"
#include "engine/IAlicaTrace.h"
#include "engine/blackboard/BlackBoard.h"
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
class RunningPlan;
class ThreadSafePlanInterface;
class AlicaEngine;
class IAlicaWorldModel;

/**
 * The base class for BasicBehaviour and BasicPlan
 */
class RunnableObject
{
protected:
    RunnableObject(IAlicaWorldModel* wm, const std::string& name = "");
    virtual ~RunnableObject() = default;
    void setEngine(AlicaEngine* engine) { _engine = engine; };
    void setConfiguration(const Configuration* conf) { _configuration = conf; };
    void setName(const std::string& name) { _name = name; };
    AlicaTime getInterval() const { return _msInterval; };
    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); };
    bool getRequiresParameters() const { return _requiresParameters; }
    void setRequiresParameters(bool requiresParameters) {_requiresParameters = requiresParameters;}
    void disableTracing() { clearFlags(Flags::TRACING_ENABLED); };
    void stop();
    void start(RunningPlan* rp);

    // This is not thread safe. Should only be called by the scheduler thread. TODO: make this private
    std::optional<std::string> getTraceContext() const { return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt; };

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
    bool _requiresParameters;
    uint8_t _flags;
    std::atomic<Counter> _execState;          // Tracks the actual executate state of the behaviour by the scheduler thread
    std::atomic<Counter> _signalState;        // Tracks the signal state from the alica main engine thread i.e. tracks start() & stop() calls
    std::atomic<RunningPlan*> _signalContext; // The running plan context when start() is called
    std::atomic<RunningPlan*> _execContext;   // The running plan context under which the behaviour is executing
    int64_t _activeRunJobId;
    BlackBoard _blackBoard;
    IAlicaWorldModel* _wm;

    virtual void doInit() = 0;
    virtual void doTerminate() = 0;

    std::optional<IAlicaTrace*> getTrace() const { return _trace ? std::optional<IAlicaTrace*>(_trace.get()) : std::nullopt; };
    void sendLogMessage(int level, const std::string& message) const;
    void setFlags(Flags flags) { _flags |= static_cast<uint8_t>(flags); }
    void clearFlags(Flags flags) { _flags &= ~static_cast<uint8_t>(flags); }
    bool areFlagsSet(Flags flags) { return (static_cast<uint8_t>(flags) & _flags) == static_cast<uint8_t>(flags); }
    bool isExecutingInContext() const
    {
        Counter sc = _signalState.load();
        Counter ec = _execState.load();
        return sc == ec && isActive(sc);
    };
    // If the counter is even then it indicates the behaviour is active i.e it is started, but not stopped yet
    constexpr static bool isActive(Counter cnt) { return !(cnt & 1); };
    ThreadSafePlanInterface getPlanContext() const;
    void setTerminatedState();
    void traceTermination();
    void initTrace();
    void traceRun();
    void traceInit(const std::string& type);
    BlackBoard& editBlackBoard() {return _blackBoard;}
    const BlackBoard& getBlackBoard() const {return _blackBoard;}
    IAlicaWorldModel* getWorldModel() { return _wm; };
};
} /* namespace alica */
