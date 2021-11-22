#include "MultiAgentTestPlan1413200862180.h"
/*PROTECTED REGION ID(eph1413200862180) ENABLED START*/
// Add additional using directives here
#include <alica_tests/TestWorldModel.h>
#include <engine/AlicaEngine.h>
#include <essentials/IdentifierConstPtr.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MultiAgentTestPlan (1413200862180)
//
// Tasks:
//   - AttackTask (1407153522080) (Entrypoint: 1413200877337)//   - DefaultTask (1225112227903) (Entrypoint: 1413200890537)//   - DefaultTask (1225112227903)
//   (Entrypoint: 1413807260446)
//
// States:
//   - OtherState (1413200877336)
//   - State1 (1413200910490)
//   - State2 (1413201030936)
//   - NewSuccessState1 (1413201164999)
//   - NewSuccessState2 (1413552736921)
//   - Idle (1413807264574)
MultiAgentTestPlan1413200862180::MultiAgentTestPlan1413200862180(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1413200862180) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MultiAgentTestPlan1413200862180::~MultiAgentTestPlan1413200862180()
{
    /*PROTECTED REGION ID(dcon1413200862180) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

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
 * Transition: 1413201368286 (1413201368286)
 *   - Comment:
 *   - Source2Dest: OtherState --> NewSuccessState1
 *
 * Precondition: 1413201370590 (1413201370590)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in OtherState:
 *   - Attack (1402488848841)
 */
bool PreCondition1413201370590::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
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
 * Transition: 1413201050743 (1413201050743)
 *   - Comment:
 *   - Source2Dest: State1 --> State2
 *
 * Precondition: 1413201052549 (1413201052549)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in State1:
 *   - Attack (1402488848841)
 */
bool PreCondition1413201052549::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
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
 * Transition: 1413201367062 (1413201367062)
 *   - Comment:
 *   - Source2Dest: State2 --> NewSuccessState2
 *
 * Precondition: 1413201367990 (1413201367990)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in State2:
 *   - Attack (1402488848841)
 */
bool PreCondition1413201367990::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
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

/*PROTECTED REGION ID(methods1413200862180) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
