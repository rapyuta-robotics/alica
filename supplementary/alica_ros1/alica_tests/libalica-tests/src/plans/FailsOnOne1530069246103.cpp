#include "FailsOnOne.h"

#include <alica_tests/SimpleSwitches.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

FailsOnOne::FailsOnOne(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void FailsOnOne::onInit() {}

std::unique_ptr<FailsOnOne> FailsOnOne::create(alica::PlanContext& context)
{
    return std::make_unique<FailsOnOne>(context);
}

std::shared_ptr<alica::UtilityFunction> FailsOnOneUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<FailsOnOneUtilityFunction> FailsOnOneUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<FailsOnOneUtilityFunction>();
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) : Is not set 1

/**
 * Available Vars:
 */
bool RunTimeCondition1530069251117::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    /*PROTECTED REGION ID(1530069251117) ENABLED START*/
    return !SimpleSwitches::isSet(1);
    /*PROTECTED REGION END*/
}

} // namespace alica::tests
