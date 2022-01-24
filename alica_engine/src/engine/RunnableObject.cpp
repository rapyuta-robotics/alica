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
        , _requiresParameters(false)
        , _activeRunJobId(-1)
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _signalState(1)
        , _execState(1)
        , _wm(wm)
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
    _engine->editScheduler().schedule(std::bind(&RunnableObject::doTerminate, this));
}

void RunnableObject::start(RunningPlan* rp)
{
    if (isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _signalContext.store(rp);
    if (_requiresParameters) {
        assert(rp->getParent());
        assert(rp->getParent()->getBasicPlan());
        const auto& wrappers = rp->getParent()->getActiveState()->getConfAbstractPlanWrappers();
        auto it =
                std::find_if(wrappers.begin(), wrappers.end(), [this](const auto& wrapper_ptr) { return wrapper_ptr->getAbstractPlan()->getName() == _name; });
        assert(it != wrappers.end());

        int64_t wrapperId = (*it)->getId();

        BasicPlan* parentPlan = rp->getParent()->getBasicPlan();
        auto& planAttachment = parentPlan->getPlanAttachment(wrapperId);
        auto initCall = [this, &planAttachment, parentPlan = parentPlan]() {
            if (!_blackboard) {
                _blackboard = std::make_shared<Blackboard>();
            }
            _blackboard->impl().clear();
            try {
                if (!planAttachment->setParameters(*parentPlan->getBlackboard(), *_blackboard)) {
                    std::cerr << "Setting parameters failed, supposedly as the context has already changed.  Plan will not be scheduled" << std::endl;
                    return;
                }
            } catch (const std::exception& e) {
                ALICA_ERROR_MSG("planAttachment: Exception during setParameters caught: "
                                << "\n"
                                << e.what());
            }
            doInit();
        };
        _engine->editScheduler().schedule(initCall);
    } else {

        BasicPlan* parentPlan = rp->getParent() ? rp->getParent()->getBasicPlan() : nullptr;
        auto initCall = [this, parentPlan = parentPlan]() {
            // Share Blackboard with parent if we have one, or start fresh otherwise
            if (parentPlan) {
                _blackboard = parentPlan->getBlackboard();
            } else {
                _blackboard = std::make_shared<Blackboard>();
            }
            doInit();
        };
        _engine->editScheduler().schedule(initCall);
    }
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

} /* namespace alica */
