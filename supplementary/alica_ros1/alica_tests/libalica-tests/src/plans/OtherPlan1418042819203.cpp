#include "OtherPlan.h"

#include "engine/USummand.h"
#include <alica_tests/TestConstantValueSummand.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

OtherPlan::OtherPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void OtherPlan::onInit() {}

std::unique_ptr<OtherPlan> OtherPlan::create(alica::PlanContext& context)
{
    return std::make_unique<OtherPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> OtherPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 0.2));
    return function;
}

std::shared_ptr<OtherPlanUtilityFunction> OtherPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<OtherPlanUtilityFunction>();
}

} // namespace alica::tests
