#include "MultiAgentTestMaster1413200842973.h"
/*PROTECTED REGION ID(eph1413200842973) ENABLED START*/
// Add additional using directives here
#include "TestWorldModel.h"
#include <engine/AlicaEngine.h>
#include <essentials/IdentifierConstPtr.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:MultiAgentTestMaster
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1413200842975
 */
std::shared_ptr<UtilityFunction> UtilityFunction1413200842973::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1413200842973) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1413201227586, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1413200842975)
 *
 * States in plan:
 *   - Init (1413200842974)
 *   - Start (1413201213955)
 *   - Finished (1413201380359)
 *
 * Variables of precondition:
 */
bool PreCondition1413201227586::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1413201226246) ENABLED START*/
    int id8 = 8;
    essentials::IdentifierConstPtr agentID8 = rp->getAlicaEngine()->getID<int>(id8);

    if (*(rp->getOwnID()) == *agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201227586();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201227586();
    }
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1413201389955, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - MultiAgentTestPlan (1413200862180)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1413200842975)
 *
 * States in plan:
 *   - Init (1413200842974)
 *   - Start (1413201213955)
 *   - Finished (1413201380359)
 *
 * Variables of precondition:
 */
bool PreCondition1413201389955::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1413201388722) ENABLED START*/
    int id8 = 8;
    essentials::IdentifierConstPtr agentID8 = rp->getAlicaEngine()->getID<int>(id8);
    if (*(rp->getOwnID()) == *agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1413201389955() /*&& rp->allChildrenStatus(PlanStatus::Success)*/;
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1413201389955() /*&& rp->allChildrenStatus(PlanStatus::Success)*/;
    }
    /*PROTECTED REGION END*/
}
} // namespace alica
