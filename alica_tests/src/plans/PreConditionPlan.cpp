#include "engine/USummand.h"
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/plans/PreConditionPlan.h>

namespace alica
{
PreConditionPlan::PreConditionPlan(PlanContext& context)
        : BasicPlan(context)
{
}

bool PreConditionPlanPreCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isPreCondition1418042929966();
}
std::shared_ptr<UtilityFunction> PreConditionPlanUtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> function = make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 1.0));
    return function;
}

} // namespace alica
