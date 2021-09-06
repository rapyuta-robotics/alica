#include "Authority/AuthorityTestMaster1414403396328.h"
/*PROTECTED REGION ID(eph1414403396328) ENABLED START*/
// Add additional using directives here
#include <memory>
using namespace std;
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  AuthorityTestMaster (1414403396328)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1414403396331)
//
// States:
//   - testState (1414403396329)
//   - Init (1414403820806)
AuthorityTestMaster1414403396328::AuthorityTestMaster1414403396328()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1414403396328) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AuthorityTestMaster1414403396328::~AuthorityTestMaster1414403396328()
{
    /*PROTECTED REGION ID(dcon1414403396328) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

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
 * Transition: 1414403840950 (1414403840950)
 *   - Comment:
 *   - Source2Dest: Init --> testState
 *
 * Precondition: 1414403842622 (1414403842622)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Init:
 */
bool PreCondition1414403842622::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1414403840950) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1414403396328) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
