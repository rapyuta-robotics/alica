#include "engine/BasicPlan.h"

#include "engine/logging/Logging.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/Transition.h"

namespace alica
{

BasicPlan::BasicPlan(PlanContext& context)
        : RunnableObject(context.worldModel, context.traceFactory, context.name)
        , _isMasterPlan(context.planModel->isMasterPlan())
        , _plan(context.planModel)
{
    if (_plan->getFrequency() < 1) {
        setInterval(0);
    } else {
        setInterval(1000 / _plan->getFrequency());
    }

    for (const State* state : _plan->getStates()) {
        for (const ConfAbstractPlanWrapper* wrapper : state->getConfAbstractPlanWrappers()) {
            addKeyMapping(wrapper->getId(), wrapper->getKeyMapping());
        }
    }
    setBlackboardBlueprint(_plan->getBlackboardBlueprint());
}

void BasicPlan::doInit()
{
    // TODO Cleanup: do this after moving trace context setting to RunningPlan
    /*if (_isMasterPlan) {
        // Immediately send out the trace for the master plan, otherwise the traces will not be available until the application ends
        _runnableObjectTracer.cleanupTraceContext();
    }*/

    try {
        onInit();
    } catch (const std::exception& e) {
        handleException("initialise", std::current_exception());
    }
}

void BasicPlan::doRun()
{
    try {
        run();
    } catch (const std::exception& e) {
        handleException("run", std::current_exception());
    }
}

void BasicPlan::doTerminate()
{
    try {
        onTerminate();
    } catch (const std::exception& e) {
        handleException("terminate", std::current_exception());
    }
}

void BasicPlan::traceAssignmentChange(const std::string& assignedEntryPoint, double oldUtility, double newUtility, size_t numberOfAgents)
{
    if (_traceFactory && getTrace()) {
        getTrace()->setLog(
                {"TaskAssignmentChange", "{\"old\": " + std::to_string(oldUtility) + ", " + "\"new\": " + std::to_string(newUtility) + ", " +
                                                 "\"agents\": " + std::to_string(numberOfAgents) + ", " + "\"ep\": \"" + assignedEntryPoint + "\"}"});
    }
}

int64_t BasicPlan::getId() const
{
    return _plan->getId();
}

} // namespace alica
