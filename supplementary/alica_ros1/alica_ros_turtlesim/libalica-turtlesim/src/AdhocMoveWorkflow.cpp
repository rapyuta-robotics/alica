#include "AdhocMoveWorkflow.h"

#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

AdhocMoveWorkflow::AdhocMoveWorkflow(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void AdhocMoveWorkflow::onInit()
{
    alica::LockedBlackboardRW bb(*getBlackboard());
    bb.set("blackboard", getBlackboard());
    bb.set("blackboardBlueprint", getBlackboardBlueprint());
    bb.set("blackboardKeys", std::vector<std::string>{"x", "y"});
}

std::unique_ptr<AdhocMoveWorkflow> AdhocMoveWorkflow::create(alica::PlanContext& context)
{
    return std::make_unique<AdhocMoveWorkflow>(context);
}

} // namespace turtlesim
