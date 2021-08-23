#include "AttackPlan1402488634525.h"
/*PROTECTED REGION ID(eph1402488634525) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  AttackPlan (1402488634525)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1402488646221)
//
// States:
//   - Attack (1402488646220)
//   - Shoot (1402489396914)
AttackPlan1402488634525::AttackPlan1402488634525()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1402488634525) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
AttackPlan1402488634525::~AttackPlan1402488634525()
{
    /*PROTECTED REGION ID(dcon1402488634525) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void AttackPlan1402488634525::run(void* msg)
{
    /*PROTECTED REGION ID(runAttackPlan1402488634525) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488646221
 */
std::shared_ptr<UtilityFunction> UtilityFunction1402488634525::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488634525) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1402489459382)
 *   - Comment:
 *   - Source2Dest: Attack --> Shoot
 *
 * Precondition: MISSING_NAME (1402489460549)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *	   - ABC (1403772834750)
 *	   - TestVar1 (1403772778288)
 *   - Quantifiers:
 *	   - MISSING_NAME (1403773214317)
 *	   - MISSING_NAME (1403773224776)
 *	   - MISSING_NAME (1403773234841)
 *	   - MISSING_NAME (1403773248357)
 *
 * Abstract Plans in Attack:
 *   - Tackle (1402489318663)
 *   - AttackOpp (1402489351885)
 */
bool PreCondition1402489460549::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489459382) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1402489460694)
 *   - Comment:
 *   - Source2Dest: Shoot --> Attack
 *
 * Precondition: ConditionNameShootAttack (1402489462088)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString: Some nice comment!
 *   - Variables:
 *	   - TestVar1 (1403772778288)
 *	   - VarTest2 (1403772797469)
 *	   - NewVar (1403772816953)
 *	   - ABC (1403772834750)
 *   - Quantifiers:
 *
 * Abstract Plans in Shoot:
 *   - Attack (1402488848841)
 */
bool PreCondition1402489462088::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489460694) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488634525) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
