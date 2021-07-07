#include "PlanFive1407153703092.h"
/*PROTECTED REGION ID(eph1407153703092) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PlanFive1407153703092
PlanFive1407153703092::PlanFive1407153703092()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1407153703092) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanFive1407153703092::~PlanFive1407153703092()
{
    /*PROTECTED REGION ID(dcon1407153703092) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153717809
 * Task: AttackTask  -> EntryPoint-ID: 1407153972059
 * Task: DefendTask  -> EntryPoint-ID: 1407153973706
 * Task: MidFieldTask  -> EntryPoint-ID: 1407153975075
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153703092::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153703092) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1407153703092) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
