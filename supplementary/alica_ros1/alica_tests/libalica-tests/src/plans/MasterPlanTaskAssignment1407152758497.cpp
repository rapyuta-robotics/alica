#include "MasterPlanTaskAssignment.h"

#include <alica_tests/SwitchEntryPointsSummand.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MasterPlanTaskAssignment::MasterPlanTaskAssignment(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MasterPlanTaskAssignment::onInit() {}

std::unique_ptr<MasterPlanTaskAssignment> MasterPlanTaskAssignment::create(alica::PlanContext& context)
{
    return std::make_unique<MasterPlanTaskAssignment>(context);
}

std::shared_ptr<alica::UtilityFunction> MasterPlanTaskAssignmentUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.1, 0.1, plan);
    SwitchEntryPointsSummand* us = new SwitchEntryPointsSummand(1.0);
    us->addEntryPoint(plan->getEntryPointByID(1407152894887)); // attack
    us->addEntryPoint(plan->getEntryPointByID(1407152902493)); // defend
    function->editUtilSummands().emplace_back(us);
    return function;
}

std::shared_ptr<MasterPlanTaskAssignmentUtilityFunction> MasterPlanTaskAssignmentUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MasterPlanTaskAssignmentUtilityFunction>();
}

} // namespace alica::tests
