#include "PlanOne1407153611768.h"
/*PROTECTED REGION ID(eph1407153611768) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanOne (1407153611768)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1407153636262)//   - AttackTask (1407153522080) (Entrypoint: 1407153791141)
//
// States:
//   - DefaultState (1407153636261)
//   - AttackState (1407153807487)
PlanOne1407153611768::PlanOne1407153611768()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1407153611768) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanOne1407153611768::~PlanOne1407153611768()
{
    /*PROTECTED REGION ID(dcon1407153611768) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void PlanOne1407153611768::run(void* msg)
{
    /*PROTECTED REGION ID(runPlanOne1407153611768) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153636262
 * Task: AttackTask  -> EntryPoint-ID: 1407153791141
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153611768::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153611768) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1407153611768) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
