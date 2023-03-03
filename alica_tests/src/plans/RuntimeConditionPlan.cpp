#include "engine/USummand.h"
#include <alica_tests/TestConstantValueSummand.h>
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/plans/RuntimeConditionPlan.h>

namespace alica
{
RuntimeConditionPlan::RuntimeConditionPlan(PlanContext& context)
        : BasicPlan(context)
{
}
bool RuntimeConditionPlanRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel = LockedBlackboardRO(*gb).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    return worldModel->isRuntimeCondition1418042967134();
}
std::shared_ptr<UtilityFunction> RuntimeConditionPlanUtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> function = make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 1.0));
    return function;
}
} // namespace alica
