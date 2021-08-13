#include "OtherPlan1418042819203.h"
/*PROTECTED REGION ID(eph1418042819203) ENABLED START*/
// Add additional using directives here
#include "TestConstantValueSummand.h"
#include "engine/USummand.h"
#include <DistXContourTest.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:OtherPlan1418042819203
OtherPlan1418042819203::OtherPlan1418042819203()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1418042819203) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
OtherPlan1418042819203::~OtherPlan1418042819203()
{
    /*PROTECTED REGION ID(dcon1418042819203) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042819206
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042819203::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042819203) ENABLED START*/

    std::shared_ptr<UtilityFunction> function = std::make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 0.2));
    return function;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418042819203) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
