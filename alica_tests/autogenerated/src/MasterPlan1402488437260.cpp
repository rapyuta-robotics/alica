#include "MasterPlan1402488437260.h"
/*PROTECTED REGION ID(eph1402488437260) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

#include <memory>

namespace alica
{
// Plan:  MasterPlan (1402488437260)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1402488437263)
//
// States:
//   - Attack (1402488437261)
//   - Defend (1402488463437)
//   - Goal (1402488470615)
//   - MidField (1402488477650)
//   - SucGoalState (1402488536570)
MasterPlan1402488437260::MasterPlan1402488437260(IAlicaWorldModel* wm)
        : DomainPlan(wm)
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
 * Transition: AttackToGoal (1402488517667)
 *   - Comment:
 *   - Source2Dest: Attack --> MidField
 *
 * Precondition: MISSING_NAME (1402488519140)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Attack:
 *   - Attack (1402488848841)
 */
bool PreCondition1402488519140::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1402488517667) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: AttackToDefend (1409218318661)
 *   - Comment:
 *   - Source2Dest: Attack --> Defend
 *
 * Precondition: MISSING_NAME (1409218319990)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Attack:
 *   - Attack (1402488848841)
 */
bool PreCondition1409218319990::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1409218318661) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: GoalToSucGoal (1402488557864)
 *   - Comment:
 *   - Source2Dest: Goal --> SucGoalState
 *
 * Precondition: MISSING_NAME (1402488558741)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Goal:
 *   - GoalPlan (1402488870347)
 */
bool PreCondition1402488558741::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1402488557864) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: MidFieldToGoal (1402488519757)
 *   - Comment:
 *   - Source2Dest: MidField --> Goal
 *
 * Precondition: MISSING_NAME (1402488520968)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in MidField:
 *   - MidFieldStandard (1402488696205)
 *   - DefendMid (1402488730695)
 *   - MidFieldPlayPlan (1402488770050)
 */
bool PreCondition1402488520968::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1402488519757) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488437260) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
