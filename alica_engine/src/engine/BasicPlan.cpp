#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"
#include "engine/PlanInterface.h"
#include "engine/scheduler/Scheduler.h"

namespace alica
{

BasicPlan::BasicPlan()
        : _ae(nullptr)
        , _configuration(nullptr)
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _activeRunJobId(-1)
        , _isMasterPlan(false)
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _signalState(1)
        , _execState(1)
        , _tracingType(TracingType::DEFAULT)
        , _customTraceContextGetter()
        , _trace()
        , _runTraced(false)
        , _initExecuted(false)
{
}

void BasicPlan::startTrace()
{
    if (!_ae->getTraceFactory()) {
        return;
    }

    switch (_tracingType) {
    case TracingType::DEFAULT: {
        auto parent = _execContext.load()->getParent();
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTraceContext().has_value()); parent = parent->getParent());
        _trace = _ae->getTraceFactory()->create(_name, parent ? parent->getBasicPlan()->getTraceContext() : std::nullopt);
        break;
    }
    case TracingType::SKIP: {
        break;
    }
    case TracingType::ROOT: {
        _trace = _ae->getTraceFactory()->create(_name);
        break;
    }
    case TracingType::CUSTOM: {
        if (!_customTraceContextGetter) {
            ALICA_ERROR_MSG("[BasicPlan] Custom tracing type specified, but no getter for the trace context is provided. Switching to default tracing type instead");
            _tracingType = TracingType::DEFAULT;
            startTrace();
            break;
        }
        _trace = _ae->getTraceFactory()->create(_name, _customTraceContextGetter(this));
        break;
    }
    }

    if (_isMasterPlan && _trace) {
        // Immediately send out the trace for the master plan, otherwise the traces will not be available till the application end
        _trace->finish();
    }
}

void BasicPlan::doInit()
{
    ++_execState;

    if (!isExecutingInContext()) {
        return;
    }
    _initExecuted = true;

    _execContext = _signalContext.exchange(nullptr);

    startTrace();

    try {
        if (_trace) {
            _trace->setLog({"Plan", "true"});
            _trace->setLog({"Init", "true"});
        }
        onInit();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-INIT" << std::endl << e.what());
    }
    // Do not schedule runJob when freq is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunJobId =  _ae->editScheduler().schedule(std::bind(&BasicPlan::doRun, this, nullptr), getInterval());
    }
}

void BasicPlan::doRun(void* msg)
{
    try {
        if (_trace && !_runTraced) {
            _trace->setLog({"Run", "true"});
            _runTraced = true;
        }
        run(msg);
    } catch (const std::exception& e) {
        std::string err = std::string("Exception caught") + std::string(" - ") + std::string(e.what());
        sendLogMessage(4, err);
    }
}

void BasicPlan::doTerminate()
{
    if (_activeRunJobId != -1) {
        _ae->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    ++_execState;

    _runTraced = false;
    if (!_initExecuted) {
        _execContext.store(nullptr);
        return;
    }
    _initExecuted = false;

    try {
        if (_trace) {
            _trace->setLog({"Terminate", "true"});
            _trace.reset();
        }
        onTerminate();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-TERMINATE" << std::endl << e.what());
    }

    _execContext.store(nullptr);
}

void BasicPlan::sendLogMessage(int level, const std::string& message) const
{
    _ae->getCommunicator().sendLogMessage(level, message);
}

void BasicPlan::start(RunningPlan* rp)
{
    if (isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _signalContext.store(rp);
    _ae->editScheduler().schedule(std::bind(&BasicPlan::doInit, this));
}

void BasicPlan::stop()
{
    if (!isActive(_signalState.load())) {
        return;
    }
    ++_signalState;
    _ae->editScheduler().schedule(std::bind(&BasicPlan::doTerminate, this));
}

ThreadSafePlanInterface BasicPlan::getPlanContext() const { return ThreadSafePlanInterface(isExecutingInContext() ? _execContext.load() : nullptr); }

} // namespace alica
