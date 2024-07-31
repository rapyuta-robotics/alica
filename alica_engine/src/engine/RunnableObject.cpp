#include "engine/RunnableObject.h"
// TODO cleanup: remove reference to BasicPlan when blackboard setup is moved to RunnningPlan
#include "engine/BasicPlan.h"
#include "engine/IAlicaCommunication.h"
#include "engine/RunningPlan.h"
#include "engine/blackboard/Blackboard.h"
#include "engine/blackboard/BlackboardUtil.h"
#include "engine/logging/Logging.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/PlanType.h"

#include <assert.h>
#include <iostream>

namespace alica
{
RunnableObject::RunnableObject(std::shared_ptr<Blackboard> globalBlackboard, const IAlicaTraceFactory* tf, const std::string& name)
        : _name(name)
        , _msInterval(AlicaTime::milliseconds(DEFAULT_MS_INTERVAL))
        , _blackboardBlueprint(nullptr)
        , _globalBlackboard(globalBlackboard)
        , _runnableObjectTracer(tf, _name)
        , _blackboard(nullptr)
        , _started(false)
{
}

void RunnableObject::sendLogMessage(int level, const std::string& message) const
{
    _communication->sendLogMessage(level, message);
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
    _runnableObjectTracer.traceTerminateCall();
    _runnableObjectTracer.cleanupTraceContext();
    cleanupBlackboard();

    _started = false;
}

void RunnableObject::start(RunningPlan* rp)
{
    if (_started) {
        return;
    }
    _started = true;

    _runningplanContext = rp;

    setupBlackboard();
    // setup the trace context after setting up the blackboard so that the blackboard can be used for setting up custom tracing
    _runnableObjectTracer.setupTraceContext(_name, _runningplanContext);

    _runnableObjectTracer.traceInitCall();
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
        _blackboard = _blackboardBlueprint ? std::make_shared<Blackboard>(_blackboardBlueprint) : _globalBlackboard;
    } else if (!getInheritBlackboard()) {
        auto parentPlan = _runningplanContext->getParent();
        auto keyMapping = _runningplanContext->getKeyMapping();

        _blackboard = std::make_shared<Blackboard>(_blackboardBlueprint); // Potentially heavy operation. TBD optimize
        BlackboardUtil::setInput(parentPlan->getBlackboard().get(), _blackboard.get(), keyMapping);
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
        auto keyMapping = _runningplanContext->getKeyMapping();
        BlackboardUtil::setOutput(parentPlan->getBlackboard().get(), _blackboard.get(), keyMapping);
    }
}

void RunnableObject::runJob()
{
    _runnableObjectTracer.traceRunCall();
    doRun();
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

void TraceRunnableObject::traceException(const std::string& exceptionOriginMethod, const std::string& details)
{
    std::string msg = "Exception thrown from: " + getName() + "'s " + exceptionOriginMethod + " method";
    if (!details.empty()) {
        msg.append(", details: " + std::move(details));
    }
    Logging::logFatal(LOGNAME) << msg;
}

void RunnableObject::handleException(const std::string& exceptionOriginMethod, std::exception_ptr eptr)
{
    std::string details;
    try {
        std::rethrow_exception(eptr);
    } catch (const std::exception& e) {
        details = e.what();
    } catch (...) {
    }
    _runnableObjectTracer.traceException(exceptionOriginMethod, details);
    _runnableObjectTracer.finishTrace();
    std::rethrow_exception(eptr);
}

// Tracing methods
void TraceRunnableObject::setTracing(TracingType type, std::function<tracing::SpanStartOptions()> customTraceContextGetter)
{
    _tracingType = type;
    _customTraceContextGetter = std::move(customTraceContextGetter);
    if (_tracingType == TracingType::CUSTOM && !_customTraceContextGetter) {
        Logging::logError(LOGNAME)
                << "Error in " << getName()
                << "Custom tracing type specified, but no getter for the trace context is provided. Switching to default tracing type instead";
        _tracingType = TracingType::DEFAULT;
    }
}

void TraceRunnableObject::setupTraceContext(const std::string& name, RunningPlan* rp)
{
    if (!_tf) {
        return;
    }

    switch (_tracingType) {
    case TracingType::DEFAULT: {
        auto parent = rp->getParent();
        for (; parent && (!parent->getBasicPlan() || !parent->getBasicPlan()->getTrace()); parent = parent->getParent())
            ;
        _trace = _tf->create(name, (parent ? std::optional<std::string>(parent->getBasicPlan()->getTrace()->context()) : std::nullopt));
        break;
    }
    case TracingType::SKIP: {
        break;
    }
    case TracingType::ROOT: {
        _trace = _tf->create(name);
        break;
    }
    case TracingType::CUSTOM: {
        _trace = _tf->create(name, _customTraceContextGetter());
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
    if (!_runTraced) {
        Logging::logInfo(LOGNAME) << "Runnable object: " << getName() << ", status: run";
        if (_trace) {
            _trace->setLog({"status", "run"});
        }
        _runTraced = true;
    }
}

void TraceRunnableObject::traceInitCall()
{
    Logging::logInfo(LOGNAME) << "Runnable object: " << getName() << ", status: init";
    if (_trace) {
        _trace->setLog({"status", "init"});
    }
}

void TraceRunnableObject::traceTerminateCall()
{
    Logging::logInfo(LOGNAME) << "Runnable object: " << getName() << ", status: terminate";
    if (_trace) {
        _trace->setLog({"status", "terminate"});
    }
}

} /* namespace alica */
