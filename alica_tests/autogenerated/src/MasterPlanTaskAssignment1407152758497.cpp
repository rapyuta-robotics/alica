#include "MasterPlanTaskAssignment1407152758497.h"
/*PROTECTED REGION ID(eph1407152758497) ENABLED START*/
#include "SwitchEntryPointsSummand.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:MasterPlanTaskAssignment1407152758497
MasterPlanTaskAssignment1407152758497::MasterPlanTaskAssignment1407152758497()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1407152758497) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MasterPlanTaskAssignment1407152758497::~MasterPlanTaskAssignment1407152758497()
{
    /*PROTECTED REGION ID(dcon1407152758497) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407152758499
 * Task: AttackTask  -> EntryPoint-ID: 1407152894887
 * Task: MidFieldTask  -> EntryPoint-ID: 1407152900425
 * Task: DefendTask  -> EntryPoint-ID: 1407152902493
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407152758497::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407152758497) ENABLED START*/
    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.1, 0.1, plan);
    SwitchEntryPointsSummand* us = new SwitchEntryPointsSummand(1.0);
    us->addEntryPoint(plan->getEntryPointByID(1407152894887)); // attack
    us->addEntryPoint(plan->getEntryPointByID(1407152902493)); // defend
    function->editUtilSummands().emplace_back(us);
    return function;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1407152758497) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
