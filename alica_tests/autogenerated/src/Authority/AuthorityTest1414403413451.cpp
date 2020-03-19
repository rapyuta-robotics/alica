#include "Authority/AuthorityTest1414403413451.h"
/*PROTECTED REGION ID(eph1414403413451) ENABLED START*/
// Add additional using directives here
#include "DummyTestSummand.h"
#include "engine/USummand.h"
#include <DistXContourTest.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:AuthorityTest
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1414403429951
 * Task: AttackTask  -> EntryPoint-ID: 1414403522424
 */
std::shared_ptr<UtilityFunction> UtilityFunction1414403413451::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1414403413451) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
