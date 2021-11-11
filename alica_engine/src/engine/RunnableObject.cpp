#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanInterface.h"
#include <alica_common_config/debug_output.h>

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject(const std::string& name)
        : _name(name)
        , _engine(nullptr)
        , _configuration(nullptr)
        , _flags(static_cast<uint8_t>(Flags::TRACING_ENABLED))
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _activeRunJobId(-1)
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _signalState(1)
        , _execState(1)
{
}

void RunnableObject::disableTracing()
{
    clearFlags(Flags::TRACING_ENABLED);
}

bool RunnableObject::isExecutingInContext() const
{
    Counter sc = _signalState.load(), ec = _execState.load();
    return sc == ec && isActive(sc);
}

bool RunnableObject::isActive(Counter cnt)
{
    return !(cnt & 1);
}

AlicaTime RunnableObject::getInterval() const
{
    return _msInterval;
}

void RunnableObject::setInterval(int32_t msInterval)
{
    _msInterval = AlicaTime::milliseconds(msInterval);
}

void RunnableObject::setName(const std::string& name)
{
    _name = name;
}

void RunnableObject::setEngine(AlicaEngine* engine)
{
    _engine = engine;
}

void RunnableObject::setConfiguration(const Configuration* conf)
{
    _configuration = conf;
}

void RunnableObject::sendLogMessage(int level, const std::string& message) const
{
    _engine->getCommunicator().sendLogMessage(level, message);
}

std::optional<IAlicaTrace*> RunnableObject::getTrace() const
{
    return _trace ? std::optional<IAlicaTrace*>(_trace.get()) : std::nullopt;
}

std::optional<std::string> RunnableObject::getTraceContext() const
{
    return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt;
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

void RunnableObject::setTerminatedState()
{
    if (_activeRunJobId != -1) {
        _engine->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    ++_execState;

    clearFlags(Flags::RUN_TRACED);
    if (!areFlagsSet(Flags::INIT_EXECUTED)) {
        _execContext.store(nullptr);
        return;
    }
    clearFlags(Flags::INIT_EXECUTED);
}

void RunnableObject::traceTermination()
{
    if (_trace) {
        _trace->setLog({"Terminate", "true"});
        _trace.reset();
    }
}

void RunnableObject::initTrace()
{
    // Get closest parent that has a trace
    if (areFlagsSet(Flags::TRACING_ENABLED) && _engine->getTraceFactory()) {
        auto parent = _execContext.load()->getParent();
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTraceContext().has_value()); parent = parent->getParent())
            ;
        _trace = _engine->getTraceFactory()->create(_name, parent ? parent->getBasicPlan()->getTraceContext() : std::nullopt);
    }
}

void RunnableObject::traceRun()
{
    if (_trace && !areFlagsSet(Flags::RUN_TRACED)) {
        _trace->setLog({"Run", "true"});
        setFlags(Flags::RUN_TRACED);
    }
}

void RunnableObject::traceInit(const std::string& type)
{
    if (_trace) {
        _trace->setLog({type, "true"});
        _trace->setLog({"Init", "true"});
    }
}
} /* namespace alica */
