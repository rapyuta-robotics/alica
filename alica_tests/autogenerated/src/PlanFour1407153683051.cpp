#include "PlanFour1407153683051.h"
/*PROTECTED REGION ID(eph1407153683051) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PlanFour1407153683051
PlanFour1407153683051::PlanFour1407153683051()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1407153683051) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanFour1407153683051::~PlanFour1407153683051()
{
    /*PROTECTED REGION ID(dcon1407153683051) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153696703
 * Task: AttackTask  -> EntryPoint-ID: 1407153949327
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153683051::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153683051) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1407153683051) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
