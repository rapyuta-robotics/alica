#include "MasterPlanTaskAssignment1407152758497.h"
/*PROTECTED REGION ID(eph1407152758497) ENABLED START*/
#include <alica_tests/SwitchEntryPointsSummand.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MasterPlanTaskAssignment (1407152758497)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1407152758499)//   - AttackTask (1407153522080) (Entrypoint: 1407152894887)//   - MidFieldTask (1407153536219)
//   (Entrypoint: 1407152900425)//   - DefendTask (1402488486725) (Entrypoint: 1407152902493)
//
// States:
//   - AttackFirst (1407152758498)
//   - MidField (1407152951886)
//   - Defend (1407152962295)
//   - Goal (1407152969078)
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

void MasterPlanTaskAssignment1407152758497::run(void* msg)
{
    /*PROTECTED REGION ID(runMasterPlanTaskAssignment1407152758497) ENABLED START*/
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
