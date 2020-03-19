#include "OtherPlan1418042819203.h"
/*PROTECTED REGION ID(eph1418042819203) ENABLED START*/
// Add additional using directives here
#include "TestConstantValueSummand.h"
#include "engine/USummand.h"
#include <DistXContourTest.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:OtherPlan
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042819206
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042819203::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042819203) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
