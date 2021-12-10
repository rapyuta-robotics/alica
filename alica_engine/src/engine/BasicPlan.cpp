#include "engine/BasicPlan.h"

#include "engine/AlicaEngine.h"
#include "engine/model/Configuration.h"
#include "engine/scheduler/Scheduler.h"

#include "engine/PlanInterface.h"

namespace alica
{

BasicPlan::BasicPlan(IAlicaWorldModel* wm)
        : RunnableObject(wm)
        , _isMasterPlan(false)
{
}

void BasicPlan::doInit()
{
    ++_execState;

    if (!isExecutingInContext()) {
        return;
    }
    _initExecuted = true;

    _execContext = _signalContext.exchange(nullptr);

    initTrace();
    if (_isMasterPlan && _trace) {
         // Immediately send out the trace for the master plan, otherwise the traces will not be available until the application ends
        _trace->finish();
    }

    try {
        traceInit("Plan");
        onInit();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-INIT" << std::endl << e.what());
    }
    // Do not schedule runJob when freq is 0.
    if (_msInterval > AlicaTime::milliseconds(0)) {
        _activeRunJobId = _engine->editScheduler().schedule(std::bind(&BasicPlan::doRun, this, nullptr), getInterval());
    }
}

void BasicPlan::doRun(void* msg)
{
    try {
        traceRun();
        run(msg);
    } catch (const std::exception& e) {
        std::string err = std::string("Exception caught") + std::string(" - ") + std::string(e.what());
        sendLogMessage(4, err);
    }
}

void BasicPlan::doTerminate()
{
    if (_activeRunJobId != -1) {
        _engine->editScheduler().cancelJob(_activeRunJobId);
        _activeRunJobId = -1;
    }
    if (setTerminatedState()) {
        return;
    }
    try {
        traceTermination();
        onTerminate();
    } catch (const std::exception& e) {
        ALICA_ERROR_MSG("[BasicPlan] Exception in Plan-TERMINATE" << std::endl << e.what());
    }

    _execContext.store(nullptr);
}



void BasicPlan::notifyAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents)
{
    if (_engine->getTraceFactory()) {
        _engine->editScheduler().schedule(
            std::bind(&BasicPlan::traceAssignmentChange, this, assignedEntryPoint, oldUtility, newUtility, numberOfAgents));
    }
}

void BasicPlan::traceAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents)
{
    if (_trace) {
        _trace->setLog({"TaskAssignmentChange", "{\"old\": " + std::to_string(oldUtility) + ", "
                                            + "\"new\": " + std::to_string(newUtility) + ", "
                                            + "\"agents\": " + std::to_string(numberOfAgents) + ", "
                                            + "\"ep\": \"" + assignedEntryPoint + "\"}"});
    }
}

} // namespace alica
