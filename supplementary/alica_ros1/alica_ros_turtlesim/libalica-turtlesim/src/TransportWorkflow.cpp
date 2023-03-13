#include "TransportWorkflow.h"

#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

TransportWorkflow::TransportWorkflow(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void TransportWorkflow::onInit()
{
    alica::LockedBlackboardRW bb(*getBlackboard());
    bb.set("blackboard", getBlackboard());
    bb.set("blackboardBlueprint", getBlackboardBlueprint());
    bb.set("blackboardKeys", std::vector<std::string>{"pick_x", "pick_y", "drop_x", "drop_y"});
}

std::unique_ptr<TransportWorkflow> TransportWorkflow::create(alica::PlanContext& context)
{
    return std::make_unique<TransportWorkflow>(context);
}

} // namespace turtlesim
