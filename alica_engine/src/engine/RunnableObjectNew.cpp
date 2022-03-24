#include "engine/RunnableObjectNew.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanInterface.h"
#include "engine/model/ConfAbstractPlanWrapper.h"

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObjectNew::RunnableObjectNew(IAlicaWorldModel* wm, const std::string& name)
        : _name(name)
        , _engine(nullptr)
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _blackboardBlueprint(nullptr)
        , _wm(wm)
        , _blackboard(nullptr)
{
}

void RunnableObjectNew::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator().sendLogMessage(level, message);
}

int64_t RunnableObjectNew::getParentWrapperId(RunningPlan* rp) const
{
    const auto& wrappers = rp->getParent()->getActiveState()->getConfAbstractPlanWrappers();
    auto it = std::find_if(wrappers.begin(), wrappers.end(), [this](const auto& wrapper_ptr) { return wrapper_ptr->getAbstractPlan()->getName() == _name; });
    assert(it != wrappers.end());
    int64_t wrapperId = (*it)->getId();
    return wrapperId;
}

void RunnableObjectNew::addKeyMapping(int64_t wrapperId, const KeyMapping* keyMapping)
{
    _keyMappings.emplace(wrapperId, keyMapping);
}

void RunnableObjectNew::stop()
{
    stopRunCalls();
    doTerminate();
    cleanupBlackboard();
    _runnableObjectTracer.cleanupTraceContext();
}

void RunnableObjectNew::start(RunningPlan* rp)
{
    _runningplanContext = rp;

    // TODO cleanup: pass trace factory in constructor. can't do now as _engine isn't available
    _runnableObjectTracer.setupTraceContext(_name, _runningplanContext, _engine->getTraceFactory());
    setupBlackboard();
    doInit();
    scheduleRunCalls();
}

void RunnableObjectNew::scheduleRunCalls()
{
    // Do not schedule repeatable run job when frequency is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunTimer = _engine->getTimerFactory().createTimer(std::bind(&RunnableObjectNew::runJob, this), _msInterval);
    }
}

void RunnableObjectNew::stopRunCalls()
{
    _activeRunTimer.reset();
}

void RunnableObjectNew::setupBlackboard()
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
        auto keyMapping = parentPlan->getKeyMapping(getParentWrapperId(_runningplanContext));

        _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint); // Potentially heavy operation. TBD optimize
        setInput(parentPlan->getBlackboard().get(), keyMapping);
    } else {
        // Inherit blackboard
        BasicPlan* parentPlan = _runningplanContext->getParent()->getBasicPlan();
        _blackboard = parentPlan->getBlackboard();
    }
}

void RunnableObjectNew::cleanupBlackboard()
{
    if (_runningplanContext->getParent() && !getInheritBlackboard()) {
        auto parentPlan = _runningplanContext->getParent();
        auto keyMapping = parentPlan->getKeyMapping(getParentWrapperId(_runningplanContext));
        setOutput(parentPlan->getBlackboard().get(), keyMapping);
    }
}

void RunnableObjectNew::runJob()
{
    _runnableObjectTracer.traceRunCall();
    doRun();
}

void RunnableObjectNew::setInput(const Blackboard* parent_bb, const KeyMapping* keyMapping)
{
    const auto lockedParentBb = LockedBlackboardRO(*parent_bb);
    auto& childBb = _blackboard->impl(); // Child not started yet, no other user exists, dont' use lock
    for (const auto& [parentKey, childKey] : keyMapping->getInputMapping()) {
        try {
            childBb.set(childKey, lockedParentBb.get(parentKey));
            ALICA_DEBUG_MSG("passing " << parentKey << " into " << childKey);
        } catch (std::exception& e) {
            ALICA_WARNING_MSG("Blackboard error passing " << parentKey << " into " << childKey << ". " << e.what());
        }
    }
}

void RunnableObjectNew::setOutput(Blackboard* parent_bb, const KeyMapping* keyMapping) const
{
    auto lockedParentBb = LockedBlackboardRW(*parent_bb);
    const auto& childBb = _blackboard->impl(); // Child is terminated, no other users exists, don't use lock
    for (const auto& [parentKey, childKey] : keyMapping->getOutputMapping()) {
        try {
            lockedParentBb.set(parentKey, childBb.get(childKey));
            ALICA_DEBUG_MSG("passing " << childKey << " into " << parentKey);
        } catch (std::exception& e) {
            ALICA_WARNING_MSG("Blackboard error passing " << childKey << " into " << parentKey << ". " << e.what());
        }
    }
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
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTraceContext()); parent = parent->getParent())
            ;
        _trace = traceFactory->create(name, (parent ? parent->getBasicPlan()->getTraceContext() : std::nullopt));
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
