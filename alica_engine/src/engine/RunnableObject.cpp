#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanInterface.h"

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
    _engine->editScheduler().schedule(std::bind(&RunnableObject::doInit, this));
}

bool RunnableObject::setTerminatedState()
{
    ++_execState;

    _runTraced = false;
    if (!_initExecuted) {
        _execContext.store(nullptr);
        return true;
    }
    _initExecuted = false;
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
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTraceContext().has_value()); parent = parent->getParent());
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

void RunnableObject::traceTermination()
{
    if (_trace) {
        _trace->setLog({"status", "terminating"});
        _trace.reset();
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
