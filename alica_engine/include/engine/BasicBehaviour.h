#pragma once

#include "engine/Assignment.h"
#include "engine/PlanInterface.h"
#include "engine/Types.h"
#include "engine/model/Behaviour.h"

#include <essentials/ITrigger.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace alica
{
namespace test
{
class TestContext;
}
class Variable;
class RunningPlan;
class Configuration;
class EntryPoint;
class AlicaEngine;

/**
 * The base class for all behaviours. All Behaviours must inherit from this class.
 */
class BasicBehaviour
{
public:
    BasicBehaviour(const std::string& name);
    virtual ~BasicBehaviour(){};
    virtual void run(void* msg) = 0;

    bool isRunningInContext(const RunningPlan* rp) const { return isExecutingInContext() && rp == _context.load(); };
    void setEngine(AlicaEngine* engine) { _engine = engine; }
    const std::string& getName() const { return _name; }

    void setBehaviour(const Behaviour* beh) { _behaviour = beh; };
    void setConfiguration(const Configuration* conf) { _configuration = conf; };

    const VariableGrp& getVariables() const { return _behaviour->getVariables(); }
    const Variable* getVariable(const std::string& name) const { return _behaviour->getVariable(name); };

    bool stop();
    bool start(RunningPlan* rp);
    void terminate() { stop(); };

    void setDelayedStart(int32_t msDelayedStart) { _msDelayedStart = AlicaTime::milliseconds(msDelayedStart); }

    void setInterval(int32_t msInterval) { _msInterval = AlicaTime::milliseconds(msInterval); }

    ThreadSafePlanInterface getPlanContext() const { return ThreadSafePlanInterface(isExecutingInContext() ? _context.load() : nullptr); }

    bool isSuccess() const { return isExecutingInContext() && _behResult.load() == BehResult::SUCCESS; };
    void setSuccess();
    bool isFailure() const { return isExecutingInContext() && _behResult.load() == BehResult::FAILURE; };
    void setFailure();

    bool getParameter(const std::string& key, std::string& valueOut) const;

    void doTrigger();
    bool isTriggeredRunFinished();

    void sendLogMessage(int level, const std::string& message) const;

    /**
     * Called after construction.
     * Override in case custom initialization has to happen after the behavior has been integrated into the engine.
     */
    virtual void init() {}

    AlicaTime getInterval() { return _msInterval; }

    bool isEventDriven() const { return _behaviour->isEventDriven(); }

protected:
    essentials::IdentifierConstPtr getOwnId() const;
    const AlicaEngine* getEngine() const { return _engine; }

    /**
     * Called whenever a basic behaviour is started, i.e., when the corresponding state is entered.
     * Override for behaviour specific initialisation. Guaranteed to be executed on the behavior's thread.
     */
    virtual void initialiseParameters() {}

    /**
     * Called whenever a basic behavior is stopped, i.e., when the corresponding state is left.
     * Override for behaviour specific termination. Guaranteed to be executed on the behavior's thread.
     */
    virtual void onTermination() {}

private:
    friend alica::test::TestContext;
    using BehState = uint64_t;
    enum class BehResult : uint8_t
    {
        UNKNOWN,
        SUCCESS,
        FAILURE
    };

    void runJob(void* msg);
    void initJob();
    void terminateJob();

    static constexpr bool isActive(BehState st) { return !(st & 1); }
    bool isExecutingInContext() const
    {
        BehState ss = _signalState.load(), es = _execState.load();
        return ss == es && isActive(ss);
    }

    /**
     * The name of this behaviour.
     */
    std::string _name;

    const Behaviour* _behaviour;
    AlicaEngine* _engine;

    /**
     * The configuration, that is set in the behaviour pool, associated with
     * this basic behaviour through its corresponding ConfAbstractPlanWrapper.
     */
    const Configuration* _configuration;

    AlicaTime _msInterval;
    AlicaTime _msDelayedStart;

    // TODO: Optimization: It should be okay (confirm it) for all accesses to atomic variables to be done using acquire-release semantics instead
    // of the current used sequentially-consistent ordering which can be a performance bottleneck because of the necessiated memory fence instruction
    // (see https://en.cppreference.com/w/cpp/atomic/memory_order#Sequentially-consistent_ordering)
    std::atomic<RunningPlan*> _context;
    std::atomic<BehState> _signalState;
    std::atomic<BehState> _execState;
    std::atomic<BehResult> _behResult;

    int64_t _activeRunJobId;
    std::atomic<bool> _triggeredJobRunning;
};
} /* namespace alica */
