#include "GSolver/GSolverTestPlan1417423757243.h"
/*PROTECTED REGION ID(eph1417423757243) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  GSolverTestPlan (1417423757243)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1417423777546)
//
// States:
//   - SolverState (1417423777544)
GSolverTestPlan1417423757243::GSolverTestPlan1417423757243(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1417423757243) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
GSolverTestPlan1417423757243::~GSolverTestPlan1417423757243()
{
    /*PROTECTED REGION ID(dcon1417423757243) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 *	- X (1417444589341)
 *	- Y (1417444593509)
 */
bool RunTimeCondition1417424512343::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1417424512343) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1417423777546
 */
std::shared_ptr<UtilityFunction> UtilityFunction1417423757243::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1417423757243) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1417423757243) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
