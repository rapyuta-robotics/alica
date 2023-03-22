#include "engine/BasicPlan.h"

#include "engine/logging/Logging.h"
#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/model/Transition.h"

namespace alica
{

BasicPlan::BasicPlan(PlanContext& context)
        : RunnableObject(context.globalBlackboard, context.traceFactory, context.name)
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
    Logging::logInfo(LOGNAME) << "Task assignment change, old utility: " << oldUtility << ", new utility: " << newUtility << ", num agents: " << numberOfAgents
                              << " assigned entry point: " << assignedEntryPoint;
    if (_traceFactory && getTrace()) {
        getTrace()->setLog({"TaskAssignmentChange",
                "{\"old utility\": " + std::to_string(oldUtility) + ", " + "\"new utility\": " + std::to_string(newUtility) + ", " +
                        "\"num agents\": " + std::to_string(numberOfAgents) + ", " + "\"assigned entry point\": \"" + assignedEntryPoint + "\"}"});
    }
}

int64_t BasicPlan::getId() const
{
    return _plan->getId();
}

std::unique_ptr<BasicPlan> BasicPlan::create(PlanContext& context)
{
    return std::make_unique<BasicPlan>(context);
}

} // namespace alica
