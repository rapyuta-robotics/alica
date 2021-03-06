#include "MultiAgentTestPlan1413200862180.h"
/*PROTECTED REGION ID(eph1413200862180) ENABLED START*/
// Add additional using directives here
#include "TestWorldModel.h"
#include <engine/AlicaEngine.h>
#include <essentials/IdentifierConstPtr.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:MultiAgentTestPlan
/**
 * Task: AttackTask  -> EntryPoint-ID: 1413200877337
 * Task: DefaultTask  -> EntryPoint-ID: 1413200890537
 * Task: DefaultTask  -> EntryPoint-ID: 1413807260446
 */
std::shared_ptr<UtilityFunction> UtilityFunction1413200862180::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1413200862180) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1413201370590, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Attack (1402488848841)
 *
 * Tasks in plan:
 *   - AttackTask (1407153522080) (Entrypoint: 1413200877337)*   - DefaultTask (1225112227903) (Entrypoint: 1413200890537)*   - DefaultTask (1225112227903)
 * (Entrypoint: 1413807260446)
 *
 * States in plan:
 *   - OtherState (1413200877336)
 *   - State1 (1413200910490)
 *   - State2 (1413201030936)
 *   - NewSuccessState1 (1413201164999)
 *   - NewSuccessState2 (1413552736921)
 *   - Idle (1413807264574)
 *
 * Variables of preconditon:
 */
bool PreCondition1413201370590::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1413201368286) ENABLED START*/
    int id8 = 8;
    essentials::IdentifierConstPtr agentID8 = rp->getAlicaEngine()->getID<int>(id8);

    if (*(rp->getOwnID()) == *agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201370590();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201370590();
    }
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1413201052549, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Attack (1402488848841)
 *
 * Tasks in plan:
 *   - AttackTask (1407153522080) (Entrypoint: 1413200877337)*   - DefaultTask (1225112227903) (Entrypoint: 1413200890537)*   - DefaultTask (1225112227903)
 * (Entrypoint: 1413807260446)
 *
 * States in plan:
 *   - OtherState (1413200877336)
 *   - State1 (1413200910490)
 *   - State2 (1413201030936)
 *   - NewSuccessState1 (1413201164999)
 *   - NewSuccessState2 (1413552736921)
 *   - Idle (1413807264574)
 *
 * Variables of preconditon:
 */
bool PreCondition1413201052549::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1413201050743) ENABLED START*/
    int id8 = 8;
    essentials::IdentifierConstPtr agentID8 = rp->getAlicaEngine()->getID<int>(id8);

    if (*(rp->getOwnID()) == *agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201052549();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201052549();
    }
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1413201367990, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Attack (1402488848841)
 *
 * Tasks in plan:
 *   - AttackTask (1407153522080) (Entrypoint: 1413200877337)*   - DefaultTask (1225112227903) (Entrypoint: 1413200890537)*   - DefaultTask (1225112227903)
 * (Entrypoint: 1413807260446)
 *
 * States in plan:
 *   - OtherState (1413200877336)
 *   - State1 (1413200910490)
 *   - State2 (1413201030936)
 *   - NewSuccessState1 (1413201164999)
 *   - NewSuccessState2 (1413552736921)
 *   - Idle (1413807264574)
 *
 * Variables of preconditon:
 */
bool PreCondition1413201367990::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1413201367062) ENABLED START*/
    int id8 = 8;
    essentials::IdentifierConstPtr agentID8 = rp->getAlicaEngine()->getID<int>(id8);

    if (*(rp->getOwnID()) == *agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201367990();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201367990();
    }
    /*PROTECTED REGION END*/
}
} // namespace alica
