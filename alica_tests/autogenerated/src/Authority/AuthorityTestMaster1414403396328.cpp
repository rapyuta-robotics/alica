#include "Authority/AuthorityTestMaster1414403396328.h"
/*PROTECTED REGION ID(eph1414403396328) ENABLED START*/
// Add additional using directives here
#include <memory>
using namespace std;
/*PROTECTED REGION END*/

namespace alica
{
// Plan:AuthorityTestMaster
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1414403396331
 */
std::shared_ptr<UtilityFunction> UtilityFunction1414403396328::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1414403396328) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1414403842622, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1414403396331)
 *
 * States in plan:
 *   - testState (1414403396329)
 *   - Init (1414403820806)
 *
 * Variables of preconditon:
 */
bool PreCondition1414403842622::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1414403840950) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
} // namespace alica
