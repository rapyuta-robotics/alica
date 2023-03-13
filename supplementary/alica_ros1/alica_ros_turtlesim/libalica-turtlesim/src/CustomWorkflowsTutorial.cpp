#include "CustomWorkflowsTutorial.h"

#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

CustomWorkflowsTutorial::CustomWorkflowsTutorial(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void CustomWorkflowsTutorial::onInit()
{
    alica::LockedBlackboardRW bb(*getBlackboard());
    bb.set("blackboard", getBlackboard());
    bb.set("blackboardBlueprint", getBlackboardBlueprint());
    bb.set("blackboardKeys", std::vector<std::string>{"workflow"});
}

std::unique_ptr<CustomWorkflowsTutorial> CustomWorkflowsTutorial::create(alica::PlanContext& context)
{
    return std::make_unique<CustomWorkflowsTutorial>(context);
}

} // namespace turtlesim
