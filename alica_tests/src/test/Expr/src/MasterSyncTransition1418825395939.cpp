#include "MasterSyncTransition1418825395939.h"
/*PROTECTED REGION ID(eph1418825395939) ENABLED START*/
// Add additional using directives here
#include <alica_tests/TestWorldModel.h>
#include <engine/AlicaEngine.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MasterSyncTransition (1418825395939)
//
// Tasks:
//   - AttackTask (1407153522080) (Entrypoint: 1418825395941)//   - DefaultTask (1225112227903) (Entrypoint: 1418825402617)
//
// States:
//   - FirstTaskFirstState (1418825395940)
//   - SecondTaskFirstState (1418825404963)
//   - FirstTaskSecondState (1418825409988)
//   - SecondTaskSecondState (1418825411686)
MasterSyncTransition1418825395939::MasterSyncTransition1418825395939(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1418825395939) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MasterSyncTransition1418825395939::~MasterSyncTransition1418825395939()
{
    /*PROTECTED REGION ID(dcon1418825395939) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: AttackTask  -> EntryPoint-ID: 1418825395941
 * Task: DefaultTask  -> EntryPoint-ID: 1418825402617
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418825395939::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418825395939) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/**
 * Transition: FirstTaskTran (1418825425833)
 *   - Comment:
 *   - Source2Dest: FirstTaskFirstState --> FirstTaskSecondState
 *
 * Precondition: MISSING_NAME (1418825427317)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in FirstTaskFirstState:
 */
bool PreCondition1418825427317::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1418825425833) ENABLED START*/
    AgentId agentID8 = 8;

    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1418825427317();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1418825427317();
    }

    /*PROTECTED REGION END*/
}

/**
 * Transition: SecondTaskTran (1418825427469)
 *   - Comment:
 *   - Source2Dest: SecondTaskFirstState --> SecondTaskSecondState
 *
 * Precondition: MISSING_NAME (1418825428924)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in SecondTaskFirstState:
 */
bool PreCondition1418825428924::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1418825427469) ENABLED START*/
    AgentId agentID8 = 8;

    if (rp->getOwnID() == agentID8) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition1418825428924();
    } else {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition1418825428924();
    }

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418825395939) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
