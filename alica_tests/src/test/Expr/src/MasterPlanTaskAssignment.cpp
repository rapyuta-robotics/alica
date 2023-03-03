#include <alica_tests/MasterPlanTaskAssignment.h>
#include <alica_tests/SwitchEntryPointsSummand.h>

namespace alica
{
MasterPlanTaskAssignment::MasterPlanTaskAssignment(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> MasterPlanTaskAssignmentUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.1, 0.1, plan);
    SwitchEntryPointsSummand* us = new SwitchEntryPointsSummand(1.0);
    us->addEntryPoint(plan->getEntryPointByID(1407152894887)); // attack
    us->addEntryPoint(plan->getEntryPointByID(1407152902493)); // defend
    function->editUtilSummands().emplace_back(us);
    return function;
}

} // namespace alica
