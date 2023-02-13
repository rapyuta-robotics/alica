#include "PreConditionPlan.h"

#include "engine/USummand.h"
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/TestWorldModel.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PreConditionPlan::PreConditionPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PreConditionPlan::onInit() {}

std::unique_ptr<PreConditionPlan> PreConditionPlan::create(alica::PlanContext& context)
{
    return std::make_unique<PreConditionPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> PreConditionPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    shared_ptr<UtilityFunction> function = make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 1.0));
    return function;
}

std::shared_ptr<PreConditionPlanUtilityFunction> PreConditionPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PreConditionPlanUtilityFunction>();
}

bool PreCondition1418042929966::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isPreCondition1418042929966();
}

} // namespace alica::tests
