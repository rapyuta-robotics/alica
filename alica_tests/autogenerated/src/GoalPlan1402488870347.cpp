#include "GoalPlan1402488870347.h"
/*PROTECTED REGION ID(eph1402488870347) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:GoalPlan1402488870347
GoalPlan1402488870347::GoalPlan1402488870347()
        : DomainPlan("GoalPlan1402488870347")
{
    /*PROTECTED REGION ID(con1402488870347) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
GoalPlan1402488870347::~GoalPlan1402488870347()
{
    /*PROTECTED REGION ID(dcon1402488870347) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
// Check of PreCondition - (Name): PreCondition, (ConditionString):  , (Comment) :

/**
 * Available Vars:
 */
bool PreCondition1402489131988::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489131988) ENABLED START*/
    //--> "PreCondition:1402489131988  not implemented";
    return true;
    /*PROTECTED REGION END*/
}
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): test, (Comment) :

/**
 * Available Vars:
 *	- test (1403773747758)
 */
bool RunTimeCondition1403773741874::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1403773741874) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
// Check of PostCondition - (Name): MISSING_NAME, (ConditionString):  , (Comment) :

/**
 * Available Vars:
 */
bool PostCondition1402489620773::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489620773) ENABLED START*/
    std::cout << "The PostCondition 1402489620773 in TerminalState Scored is not implement yet!" << std::endl;
    std::cout << "However, PostConditions are a feature that makes sense in the context of planning, which is not supported by ALICA, yet! So don't worry.'"
              << std::endl;
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488881800
 */
std::shared_ptr<UtilityFunction> UtilityFunction1402488870347::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488870347) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488881800)
 *
 * States in plan:
 *   - Shoot (1402488881799)
 *   - Miss (1402489152217)
 *   - Scored (1402489192198)
 *
 * Variables of precondition:
 */
bool PreCondition1402489174338::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489173167) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488881800)
 *
 * States in plan:
 *   - Shoot (1402488881799)
 *   - Miss (1402489152217)
 *   - Scored (1402489192198)
 *
 * Variables of precondition:
 */
bool PreCondition1402489206278::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489205153) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488881800)
 *
 * States in plan:
 *   - Shoot (1402488881799)
 *   - Miss (1402489152217)
 *   - Scored (1402489192198)
 *
 * Variables of precondition:
 */
bool PreCondition1402489218027::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489216617) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488870347) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
