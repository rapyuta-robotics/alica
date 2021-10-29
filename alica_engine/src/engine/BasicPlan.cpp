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
        , _signalContext(nullptr)
        , _execContext(nullptr)
        , _signalState(1)
        , _execState(1)
        , _flags(static_cast<uint8_t>(Flags::TRACING_ENABLED))
{
}

void BasicPlan::doInit()
{
    ++_execState;

    if (!isExecutingInContext()) {
        return;
    }
    setFlags(Flags::INIT_EXECUTED);

    _execContext = _signalContext.exchange(nullptr);

    if (areFlagsSet(Flags::TRACING_ENABLED) && _ae->getTraceFactory()) {
        auto parent = _execContext.load()->getParent();
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTraceContext().has_value()); parent = parent->getParent());
        _trace = _ae->getTraceFactory()->create(_name, parent ? parent->getBasicPlan()->getTraceContext() : std::nullopt);
    }

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
        if (_trace && !areFlagsSet(Flags::RUN_TRACED)) {
            _trace->setLog({"Run", "true"});
            setFlags(Flags::RUN_TRACED);
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

    clearFlags(Flags::RUN_TRACED);
    if (!areFlagsSet(Flags::INIT_EXECUTED)) {
        _execContext.store(nullptr);
        return;
    }
    clearFlags(Flags::INIT_EXECUTED);

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

std::optional<IAlicaTrace*> BasicPlan::getTrace() const
{
    return _trace ? std::optional<IAlicaTrace*>(_trace.get()) : std::nullopt;
}

std::optional<std::string> BasicPlan::getTraceContext() const
{
    return _trace ? std::optional<std::string>(_trace->context()) : std::nullopt;
}

} // namespace alica
