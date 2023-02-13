#include "GoalPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

GoalPlan::GoalPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void GoalPlan::onInit() {}

std::unique_ptr<GoalPlan> GoalPlan::create(alica::PlanContext& context)
{
    return std::make_unique<GoalPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> GoalPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<GoalPlanUtilityFunction> GoalPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<GoalPlanUtilityFunction>();
}

bool PreCondition1402489131988::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

bool RunTimeCondition1403773741874::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}
bool PostCondition1402489620773::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    std::cout << "The PostCondition 1402489620773 in TerminalState Scored is not implement yet!" << std::endl;
    std::cout << "However, PostConditions are a feature that makes sense in the context of planning, which is not supported by ALICA, yet! So don't worry.'"
              << std::endl;
    return false;
}

} // namespace alica::tests
