#include "PlanThree1407153663917.h"
/*PROTECTED REGION ID(eph1407153663917) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PlanThree1407153663917
PlanThree1407153663917::PlanThree1407153663917()
        : DomainPlan("PlanThree1407153663917")
{
    /*PROTECTED REGION ID(con1407153663917) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanThree1407153663917::~PlanThree1407153663917()
{
    /*PROTECTED REGION ID(dcon1407153663917) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153675525
 * Task: MidFieldTask  -> EntryPoint-ID: 1407153896585
 * Task: DefendTask  -> EntryPoint-ID: 1407153899241
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153663917::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153663917) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
