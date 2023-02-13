#include "ConstraintTestPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

ConstraintTestPlan::ConstraintTestPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void ConstraintTestPlan::onInit() {}

std::unique_ptr<ConstraintTestPlan> ConstraintTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<ConstraintTestPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> ConstraintTestPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<ConstraintTestPlanUtilityFunction> ConstraintTestPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<ConstraintTestPlanUtilityFunction>();
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 *	- X (1414068572540)
 *	- Y (1414068576620)
 */
bool RunTimeCondition1414068566297::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(1414068566297) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}

} // namespace alica::tests
