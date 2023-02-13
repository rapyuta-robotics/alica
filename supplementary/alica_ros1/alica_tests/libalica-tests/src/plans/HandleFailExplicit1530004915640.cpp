#include "HandleFailExplicit.h"

#include <alica_tests/SimpleSwitches.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

HandleFailExplicit::HandleFailExplicit(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void HandleFailExplicit1530004915640::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("aToBSwitch", 0);
    bb.set<int64_t>("cToDSwitch", 2);
}

std::unique_ptr<HandleFailExplicit> HandleFailExplicit::create(alica::PlanContext& context)
{
    return std::make_unique<HandleFailExplicit>(context);
}

std::shared_ptr<alica::UtilityFunction> HandleFailExplicitUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<HandleFailExplicitUtilityFunction> HandleFailExplicitUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<HandleFailExplicitUtilityFunction>();
}

} // namespace alica::tests
