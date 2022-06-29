#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
// TODO cleanup: remove reference to BasicPlan when blackboard setup is moved to RunnningPlan
#include "engine/BasicPlan.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/PlanType.h"

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject(IAlicaWorldModel* wm, const std::string& name)
        : _name(name)
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _blackboardBlueprint(nullptr)
        , _wm(wm)
        , _blackboard(nullptr)
        , _started(false)
{
}

void RunnableObject::sendLogMessage(int level, const std::string& message) const
{
    _communication->sendLogMessage(level, message);
}

int64_t RunnableObject::getParentWrapperId(RunningPlan* rp) const
{
    const auto& wrappers = rp->getParent()->getActiveState()->getConfAbstractPlanWrappers();
    auto it = std::find_if(wrappers.begin(), wrappers.end(), [this](const auto& wrapper_ptr) {
        if (const auto planType = dynamic_cast<const PlanType*>(wrapper_ptr->getAbstractPlan()); planType) {
            const auto& plans = planType->getPlans();
            return std::find_if(plans.begin(), plans.end(), [this](const auto& plan) { return plan->getName() == _name; }) != plans.end();
        } else {
            return wrapper_ptr->getAbstractPlan()->getName() == _name;
        }
    });
    assert(it != wrappers.end());
    int64_t wrapperId = (*it)->getId();
    return wrapperId;
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

    _runnableObjectTracer.setupTraceContext(_name, _runningplanContext, _traceFactory);
    setupBlackboard();
    doInit();
    scheduleRunCalls();
}

void RunnableObject::scheduleRunCalls()
{
    // Do not schedule repeatable run job when frequency is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunTimer = _timerFactory->createTimer(std::bind(&RunnableObject::runJob, this), _msInterval);
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
        auto keyMapping = parentPlan->getKeyMapping(getParentWrapperId(_runningplanContext));

        _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint); // Potentially heavy operation. TBD optimize
        setInput(parentPlan->getBlackboard().get(), keyMapping);
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
        auto keyMapping = parentPlan->getKeyMapping(getParentWrapperId(_runningplanContext));
        setOutput(parentPlan->getBlackboard().get(), keyMapping);
    }
}

void RunnableObject::runJob()
{
    _runnableObjectTracer.traceRunCall();
    doRun();
}

void RunnableObject::setInput(const Blackboard* parent_bb, const KeyMapping* keyMapping)
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

void RunnableObject::setOutput(Blackboard* parent_bb, const KeyMapping* keyMapping) const
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

void RunnableObject::setAlicaCommunication(const IAlicaCommunication* communication)
{
    _communication = communication;
}
void RunnableObject::setAlicaTraceFactory(const IAlicaTraceFactory* traceFactory)
{
    _traceFactory = traceFactory;
}
void RunnableObject::setAlicaTimerFactory(const IAlicaTimerFactory* timerFactory)
{
    _timerFactory = timerFactory;
}
void RunnableObject::setPlanBase(PlanBase* planBase)
{
    _planBase = planBase;
}
void RunnableObject::setTeamManager(const TeamManager* teamManager)
{
    _teamManager = teamManager;
}

const TeamManager& RunnableObject::getTeamManager() const
{
    return *_teamManager;
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
