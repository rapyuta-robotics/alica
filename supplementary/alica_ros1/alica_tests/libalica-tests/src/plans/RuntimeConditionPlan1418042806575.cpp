#include "RuntimeConditionPlan.h"

#include "engine/USummand.h"
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/TestWorldModel.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

RuntimeConditionPlan::RuntimeConditionPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void RuntimeConditionPlan::onInit() {}

std::unique_ptr<RuntimeConditionPlan> RuntimeConditionPlan::create(alica::PlanContext& context)
{
    return std::make_unique<RuntimeConditionPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> RuntimeConditionPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    shared_ptr<UtilityFunction> function = make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 1.0));
    return function;
}

std::shared_ptr<RuntimeConditionPlanUtilityFunction> RuntimeConditionPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<RuntimeConditionPlanUtilityFunction>();
}

bool RunTimeCondition1418042967134::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isRuntimeCondition1418042967134();
}

} // namespace alica::tests
