#include "MasterPlan1402488437260.h"
/*PROTECTED REGION ID(eph1402488437260) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:MasterPlan1402488437260
MasterPlan1402488437260::MasterPlan1402488437260()
        : DomainPlan("MasterPlan1402488437260")
{
    /*PROTECTED REGION ID(con1402488437260) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MasterPlan1402488437260::~MasterPlan1402488437260()
{
    /*PROTECTED REGION ID(dcon1402488437260) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488437263
 */
std::shared_ptr<UtilityFunction> UtilityFunction1402488437260::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488437260) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Attack (1402488848841)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488437263)
 *
 * States in plan:
 *   - Attack (1402488437261)
 *   - Defend (1402488463437)
 *   - Goal (1402488470615)
 *   - MidField (1402488477650)
 *   - SucGoalState (1402488536570)
 *
 * Variables of precondition:
 */
bool PreCondition1402488519140::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402488517667) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Attack (1402488848841)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488437263)
 *
 * States in plan:
 *   - Attack (1402488437261)
 *   - Defend (1402488463437)
 *   - Goal (1402488470615)
 *   - MidField (1402488477650)
 *   - SucGoalState (1402488536570)
 *
 * Variables of precondition:
 */
bool PreCondition1409218319990::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1409218318661) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - GoalPlan (1402488870347)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488437263)
 *
 * States in plan:
 *   - Attack (1402488437261)
 *   - Defend (1402488463437)
 *   - Goal (1402488470615)
 *   - MidField (1402488477650)
 *   - SucGoalState (1402488536570)
 *
 * Variables of precondition:
 */
bool PreCondition1402488558741::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402488557864) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - MidFieldStandard (1402488696205)
 *   - DefendMid (1402488730695)
 *   - MidFieldPlayPlan (1402488770050)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488437263)
 *
 * States in plan:
 *   - Attack (1402488437261)
 *   - Defend (1402488463437)
 *   - Goal (1402488470615)
 *   - MidField (1402488477650)
 *   - SucGoalState (1402488536570)
 *
 * Variables of precondition:
 */
bool PreCondition1402488520968::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402488519757) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
} // namespace alica
