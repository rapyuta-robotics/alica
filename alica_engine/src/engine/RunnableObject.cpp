#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
// TODO cleanup: remove reference to BasicPlan when blackboard setup is moved to RunnningPlan
#include "engine/BasicPlan.h"
#include "engine/blackboard/BlackboardUtil.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/PlanType.h"

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject(IAlicaWorldModel* wm, const std::string& name)
        : _name(name)
        , _engine(nullptr)
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _blackboardBlueprint(nullptr)
        , _wm(wm)
        , _blackboard(nullptr)
        , _started(false)
{
}

void RunnableObject::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator().sendLogMessage(level, message);
}

void RunnableObject::addKeyMapping(int64_t wrapperId, const KeyMapping* keyMapping)
{
    _keyMappings.emplace(wrapperId, keyMapping);
}

void RunnableObject::stop()
{
    if (!_started) {
        return;
    }

    stopRunCalls();
    doTerminate();
    cleanupBlackboard();
    _runnableObjectTracer.cleanupTraceContext();

    _started = false;
}

void RunnableObject::start(RunningPlan* rp)
{
    if (_started) {
        return;
    }
    _started = true;

    _runningplanContext = rp;

    // TODO cleanup: pass trace factory in constructor. can't do now as _engine isn't available
    _runnableObjectTracer.setupTraceContext(_name, _runningplanContext, _engine->getTraceFactory());
    setupBlackboard();
    doInit();
    scheduleRunCalls();
}

void RunnableObject::scheduleRunCalls()
{
    // Do not schedule repeatable run job when frequency is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunTimer = _engine->getTimerFactory().createTimer(std::bind(&RunnableObject::runJob, this), _msInterval);
    }
}

void RunnableObject::stopRunCalls()
{
    _activeRunTimer.reset();
}

void RunnableObject::setupBlackboard()
{
    if (!_runningplanContext->getParent() || !_runningplanContext->getParent()->getBasicPlan()) {
        if (!_blackboard) {
            if (_blackboardBlueprint) {
                _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint); // Potentially heavy operation. TBD optimize
            } else {
                _blackboard = std::make_shared<Blackboard>();
            }
        }
    } else if (!getInheritBlackboard()) {
        auto parentPlan = _runningplanContext->getParent();
        auto keyMapping = parentPlan->getKeyMapping(_runningplanContext->getParentWrapperId(_runningplanContext));

        _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint); // Potentially heavy operation. TBD optimize
        BlackboardUtil::setInput(parentPlan->getBlackboard().get(), _blackboard.get(), keyMapping);
    } else {
        // Inherit blackboard
        BasicPlan* parentPlan = _runningplanContext->getParent()->getBasicPlan();
        _blackboard = parentPlan->getBlackboard();
    }
}

void RunnableObject::cleanupBlackboard()
{
    if (_runningplanContext->getParent() && !getInheritBlackboard()) {
        auto parentPlan = _runningplanContext->getParent();
        auto keyMapping = parentPlan->getKeyMapping(_runningplanContext->getParentWrapperId(_runningplanContext));
        BlackboardUtil::setOutput(parentPlan->getBlackboard().get(), _blackboard.get(), keyMapping);
    }
}

void RunnableObject::runJob()
{
    _runnableObjectTracer.traceRunCall();
    doRun();
}

// Tracing methods
void TraceRunnableObject::setTracing(TracingType type, std::function<std::optional<std::string>()> customTraceContextGetter)
{
    _tracingType = type;
    _customTraceContextGetter = std::move(customTraceContextGetter);
    if (_tracingType == TracingType::CUSTOM && !_customTraceContextGetter) {
        ALICA_ERROR_MSG("Custom tracing type specified, but no getter for the trace context is provided. Switching to default tracing type instead");
        _tracingType = TracingType::DEFAULT;
    }
}

void TraceRunnableObject::setupTraceContext(const std::string& name, RunningPlan* rp, const IAlicaTraceFactory* traceFactory)
{
    if (!traceFactory) {
        return;
    }

    switch (_tracingType) {
    case TracingType::DEFAULT: {
        auto parent = rp->getParent();
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTrace()); parent = parent->getParent())
            ;
        _trace = traceFactory->create(name, (parent ? std::optional<std::string>(parent->getBasicPlan()->getTrace()->context()) : std::nullopt));
        break;
    }
    case TracingType::SKIP: {
        break;
    }
    case TracingType::ROOT: {
        _trace = traceFactory->create(name);
        break;
    }
    case TracingType::CUSTOM: {
        _trace = traceFactory->create(name, _customTraceContextGetter());
        break;
    }
    }
}

void TraceRunnableObject::cleanupTraceContext()
{
    _trace.reset();
}

void TraceRunnableObject::traceRunCall()
{
    if (_trace && !_runTraced) {
        _trace->setLog({"status", "run"});
        _runTraced = true;
    }
}
} /* namespace alica */
