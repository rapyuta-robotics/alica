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

std::pair<BasicPlan*, KeyMapping> RunnableObject::getParentPlanAndKeyMapping(RunningPlan* rp) const
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
    if (rp->getParent()) {
        if (!getInheritBlackboard()) {
            auto [parentPlan, keyMapping] = getParentPlanAndKeyMapping(rp);
            auto terminateCall = [this, parentPlan, keyMapping]() {
                assert(_blackboard);
                keyMapping.setOutput(parentPlan->getBlackboard().get(), _blackboard.get());
                doTerminate();
            };
            _engine->editScheduler().schedule(terminateCall);
            return;
        }
    }
    _engine->editScheduler().schedule([this]() { doTerminate(); });
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
                    _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint);
                } else {
                    _blackboard = std::make_shared<Blackboard>();
                }
            }
            doInit();
        };
    } else if (!getInheritBlackboard()) {
        auto [parentPlan, keyMapping] = getParentPlanAndKeyMapping(rp);
        initCall = [this, parentPlan, keyMapping]() {
            _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint);
            keyMapping.setInput(parentPlan->getBlackboard().get(), _blackboard.get());
            doInit();
        };
    } else if (getInheritBlackboard()) {
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
} /* namespace alica */
