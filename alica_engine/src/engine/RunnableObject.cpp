#include "engine/RunnableObject.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanInterface.h"

#include <alica_common_config/debug_output.h>

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject(IAlicaWorldModel* wm, const std::string& name)
        : _wm(wm)
        , _name(name)
        , _engine(nullptr)
        , _configuration(nullptr)
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _tracingType(TracingType::DEFAULT)
        , _customTraceContextGetter()
        , _trace()
        , _execState(1)
        , _signalState(1)
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _initExecuted(false)
        , _activeRunJobId(-1)
        , _runTraced(false)
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

void RunnableObject::doStop()
{
    if (!isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
}

void RunnableObject::doStart(RunningPlan* rp)
{
    if (isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _signalContext.store(rp);
}

void RunnableObject::startTrace()
{
    if (!_engine->getTraceFactory()) {
        return;
    }

    switch (_tracingType) {
    case TracingType::DEFAULT: {
        auto parent = _execContext.load()->getParent();
        for (; parent && !parent->getBasicPlan()->getTraceContext().has_value(); parent = parent->getParent());
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

void RunnableObject::doInit()
{
   ++_execState;

    if (!isExecutingInContext()) {
        return;
    }
    _initExecuted = true;

    // There is a possible race condition here in the sense that the _execState can be behind the _signalState
    // and yet this behaviour can execute in the _signalState's RunningPlan context. However this is harmless
    // except for creating a superflous trace, since all other methods are guarded by isExecutingInContext()
    // which will return false in all such cases.
    // Atomically set the signal context to nullptr so that the RunningPlan instance can be deleted
    // when the behaviour is terminated
    _execContext = _signalContext.exchange(nullptr);

    startTrace();

    try {
        if (_trace) {
            _trace->setLog({"status", "initializing"});
        }
        onInit_();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[RunnableObject] Exception in: " << getName() << "\n" << e.what());
    }
}

void RunnableObject::doRun()
{
    // TODO: get rid of msg
    try {
        if (_trace && !_runTraced) {
            _trace->setLog({"status", "running"});
            _runTraced = true;
        }
        onRun_();
    } catch (const std::exception& e) {
        std::string err = std::string("Exception caught:  ") + getName() + std::string(" - ") + std::string(e.what());
        sendLogMessage(4, err);
    }
}

void RunnableObject::doTerminate()
{
    ++_execState;

    _runTraced = false;
    if (!_initExecuted) {
        // Reset the execution context so that the RunningPlan instance can be deleted
        _execContext.store(nullptr);
        return;
    }
    _initExecuted = false;

    // Intentionally call onTermination() at the end. This prevents setting success/failure from this method
    try {
        if (_trace) {
            _trace->setLog({"status", "terminating"});
        }
        onTerminate_();
        _trace.reset();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicBehaviour] Exception in Behaviour-TERMINATE of: " << getName() << "\n" << e.what());
    }

    // Reset the execution context so that the RunningPlan instance can be deleted
    _execContext.store(nullptr);
}

} /* namespace alica */
