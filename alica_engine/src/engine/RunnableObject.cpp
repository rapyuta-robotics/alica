#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanInterface.h"
#include "engine/model/ConfAbstractPlanWrapper.h"

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject(IAlicaWorldModel* wm, const std::string& name)
        : _name(name)
        , _engine(nullptr)
        , _configuration(nullptr)
        , _tracingType(TracingType::DEFAULT)
        , _runTraced(false)
        , _initExecuted(false)
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _activeRunJobId(-1)
        , _blackboardBlueprint(nullptr)
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _signalState(1)
        , _execState(1)
        , _wm(wm)
        , _blackboard(nullptr)
{
}

void RunnableObject::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator().sendLogMessage(level, message);
}

ThreadSafePlanInterface RunnableObject::getPlanContext() const
{
    return ThreadSafePlanInterface(isExecutingInContext() ? _execContext.load() : nullptr);
}

void RunnableObject::stop()
{
    if (!isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _engine->editScheduler().schedule([this]() { doTerminate(); });
}

std::pair<BasicPlan*, const KeyMapping*> RunnableObject::getParentPlanAndKeyMapping(RunningPlan* rp) const
{
    assert(rp->getParent()->getBasicPlan());
    BasicPlan* parentPlan = rp->getParent()->getBasicPlan();
    const auto& wrappers = rp->getParent()->getActiveState()->getConfAbstractPlanWrappers();
    auto it = std::find_if(wrappers.begin(), wrappers.end(), [this](const auto& wrapper_ptr) { return wrapper_ptr->getAbstractPlan()->getName() == _name; });
    assert(it != wrappers.end());
    int64_t wrapperId = (*it)->getId();
    const auto keyMapping = parentPlan->getKeyMapping(wrapperId);
    return {parentPlan, keyMapping};
}

void RunnableObject::stop(RunningPlan* rp)
{
    if (!isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    if (rp->getParent() && !getInheritBlackboard()) {
        auto [parentPlan, keyMapping] = getParentPlanAndKeyMapping(rp);
        auto terminateCall = [this, parentPlan, keyMapping]() {
            assert(_blackboard);
            setOutput(parentPlan->getBlackboard().get(), keyMapping);
            doTerminate();
        };
        _engine->editScheduler().schedule(terminateCall);
    } else {
        _engine->editScheduler().schedule([this]() { doTerminate(); });
    }
}

void RunnableObject::start(RunningPlan* rp)
{
    if (isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _signalContext.store(rp);
    std::function<void()> initCall;
    if (!rp->getParent() || !rp->getParent()->getBasicPlan()) {
        initCall = [this]() {
            if (!_blackboard) {
                if (_blackboardBlueprint) {
                    _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint); // Potentially heavy operation. TBD optimize
                } else {
                    _blackboard = std::make_shared<Blackboard>();
                }
            }
            doInit();
        };
    } else if (!getInheritBlackboard()) {
        auto [parentPlan, keyMapping] = getParentPlanAndKeyMapping(rp);
        initCall = [this, parentPlan, keyMapping]() {
            _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint); // Potentially heavy operation. TBD optimize
            setInput(parentPlan->getBlackboard().get(), keyMapping);
            doInit();
        };
    } else {
        // Inherit blackboard
        BasicPlan* parentPlan = rp->getParent()->getBasicPlan();
        initCall = [this, parentPlan]() {
            _blackboard = parentPlan->getBlackboard();
            doInit();
        };
    }
    _engine->editScheduler().schedule(initCall);
}

bool RunnableObject::setTerminatedState()
{
    ++_execState;

    _runTraced = false;
    if (!_initExecuted.load()) {
        _execContext.store(nullptr);
        return true;
    }
    _initExecuted.store(false);
    return false;
}

void RunnableObject::initTrace()
{
    if (!_engine->getTraceFactory()) {
        return;
    }

    switch (_tracingType) {
    case TracingType::DEFAULT: {
        auto parent = _execContext.load()->getParent();
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTraceContext().has_value()); parent = parent->getParent())
            ;
        _trace = _engine->getTraceFactory()->create(_name, parent ? parent->getBasicPlan()->getTraceContext() : std::nullopt);
        break;
    }
    case TracingType::SKIP: {
        break;
    }
    case TracingType::ROOT: {
        _trace = _engine->getTraceFactory()->create(_name);
        break;
    }
    case TracingType::CUSTOM: {
        _trace = _engine->getTraceFactory()->create(_name, _customTraceContextGetter());
        break;
    }
    }
}

void RunnableObject::traceRun()
{
    if (_trace && !_runTraced) {
        _trace->setLog({"status", "running"});
        _runTraced = true;
    }
}

void RunnableObject::traceInit(const std::string& type)
{
    if (_trace) {
        _trace->setTag(type, "true");
        _trace->setLog({"status", "initializing"});
    }
}

void RunnableObject::setBlackboardBlueprint(const BlackboardBlueprint* blackboard)
{
    _blackboardBlueprint = blackboard;
}

void RunnableObject::setInput(const Blackboard* parent_bb, const KeyMapping* keyMapping)
{
    const auto lockedParentBb = LockedBlackboardRO(*parent_bb);
    auto& childBb = _blackboard->impl(); // Child not started yet, no other user exists, dont' use lock
    for (const auto& [parentKey, childKey] : keyMapping->getInputMapping()) {
        childBb.set(childKey, lockedParentBb.get(parentKey));
        ALICA_DEBUG_MSG("passing " << parentKey << " into " << childKey);
    }
}

void RunnableObject::setOutput(Blackboard* parent_bb, const KeyMapping* keyMapping) const
{
    auto lockedParentBb = LockedBlackboardRW(*parent_bb);
    const auto& childBb = _blackboard->impl(); // Child is terminated, no other users exists, don't use lock
    for (const auto& [parentKey, childKey] : keyMapping->getOutputMapping()) {
        lockedParentBb.set(parentKey, childBb.get(childKey));
        ALICA_DEBUG_MSG("passing " << childKey << " into " << parentKey);
    }
}
} /* namespace alica */
