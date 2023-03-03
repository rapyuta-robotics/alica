#include "engine/USummand.h"
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/plans/OtherPlan.h>

namespace alica
{
OtherPlan::OtherPlan(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> OtherPlanUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 0.2));
    return function;
}
} // namespace alica
