#pragma once

#include "engine/Assignment.h"
#include "engine/IAlicaTrace.h"
#include "engine/RunnableObject.h"
#include "engine/Types.h"
#include "engine/model/Behaviour.h"

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
class EntryPoint;

struct BehaviourContext
{
    IAlicaWorldModel* worldModel;
    const std::string name;
    const Behaviour* behaviourModel;
    const IAlicaTraceFactory* traceFactory;
};

/**
 * The base class for all behaviours. All Behaviours must inherit from this class.
 */
class BasicBehaviour : private RunnableObject
{
public:
    BasicBehaviour(BehaviourContext& context);
    virtual ~BasicBehaviour(){};

    // Use of private inheritance and explicitly making members public
    // to share code between BasicBehaviour and Runnable object but not expose internals to further derived classes
    using RunnableObject::getBlackboard;
    using RunnableObject::getInheritBlackboard;
    using RunnableObject::getName;
    using RunnableObject::getPlanContext;
    using RunnableObject::getWorldModel;
    using RunnableObject::setEngine;
    using RunnableObject::setInterval;
    using RunnableObject::start;
    using RunnableObject::stop;
    using RunnableObject::TracingType;

    virtual void run(void* msg) = 0;
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
    const VariableGrp& getVariables() const { return _behaviour->getVariables(); }
    const Variable* getVariable(const std::string& name) const { return _behaviour->getVariable(name); };
    int64_t getId() const { return _behaviour->getId(); }

    bool isSuccess() const { return _behResult.load() == BehResult::SUCCESS; };
    bool isFailure() const { return _behResult.load() == BehResult::FAILURE; };

    void doTrigger();

protected:
    using RunnableObject::getTrace;
    using RunnableObject::getTraceFactory;

    AgentId getOwnId() const;
    const AlicaEngine* getEngine() const { return _engine; }

    void setTracing(TracingType type, std::function<std::optional<std::string>(const BasicBehaviour*)> customTraceContextGetter = {})
    {
        if (customTraceContextGetter) {
            RunnableObject::setTracing(
                    type, [this, customTraceContextGetter = std::move(customTraceContextGetter)]() { return customTraceContextGetter(this); });
        } else {
            RunnableObject::setTracing(type, {});
        }
    }

    void setSuccess();
    void setFailure();

private:
    friend alica::test::TestContext;
    enum class BehResult : uint8_t
    {
        UNKNOWN,
        SUCCESS,
        FAILURE
    };

    void doInit() override;
    void doRun() override;
    void doTerminate() override;

    bool isEventDriven() const { return _behaviour->isEventDriven(); }
    bool isTriggeredRunFinished();
    void setResult(BehResult result);

    const Behaviour* _behaviour;
    std::atomic<BehResult> _behResult;
    std::atomic<bool> _triggeredJobRunning;
};
} /* namespace alica */
