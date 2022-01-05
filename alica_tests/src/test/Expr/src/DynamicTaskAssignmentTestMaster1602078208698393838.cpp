#include "DynamicTaskAssignmentTestMaster1602078208698393838.h"
/*PROTECTED REGION ID(eph1602078208698393838) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DynamicTaskAssignmentTestMaster (1602078208698393838)
//
// Tasks:
//   - DynamicTaskTestEP (3903894018484081749) (Entrypoint: 699381937789438517)
//
// States:
//   - Start (751302000461175045)
//   - Finish (1317277234576050904)
//   - Init (4467904887554008050)
//   - 3235149896384117046 (3235149896384117046)
DynamicTaskAssignmentTestMaster1602078208698393838::DynamicTaskAssignmentTestMaster1602078208698393838(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1602078208698393838) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DynamicTaskAssignmentTestMaster1602078208698393838::~DynamicTaskAssignmentTestMaster1602078208698393838()
{
    /*PROTECTED REGION ID(dcon1602078208698393838) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DynamicTaskTestEP  -> EntryPoint-ID: 699381937789438517
 */
std::shared_ptr<UtilityFunction> UtilityFunction1602078208698393838::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1602078208698393838) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 3712615202042019043 (3712615202042019043)
 *   - Comment:
 *   - Source2Dest: Start --> Finish
 *
 * Precondition: 4344644064496100420 (4344644064496100420)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Start:
 *   - DynamicTaskLA (3337489358878214836)
 *   - DynamicTaskAssignmentTest (2252865124432942907)
 */
bool PreCondition4344644064496100420::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(3712615202042019043) ENABLED START*/
    std::cout << "The PreCondition 4344644064496100420 in Transition '3712615202042019043' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 4266666033623620026 (4266666033623620026)
 *   - Comment:
 *   - Source2Dest: Init --> Start
 *
 * Precondition: InitDone (4496654201854254411)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Init:
 */
bool PreCondition4496654201854254411::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(4266666033623620026) ENABLED START*/
    if (alicaTests::TestWorldModel::getCurAgent() != rp->getOwnID()) {
        return false;
    }
    if (alicaTests::TestWorldModel::getCurAgent() == 9) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition4496654201854254411();
    } else if (alicaTests::TestWorldModel::getCurAgent() == 8) {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition4496654201854254411();
    }
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1127748231963620498 (1127748231963620498)
 *   - Comment:
 *   - Source2Dest: Init --> 3235149896384117046
 *
 * Precondition: 3126176581533900616 (3126176581533900616)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Init:
 */
bool PreCondition3126176581533900616::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1127748231963620498) ENABLED START*/
    if (alicaTests::TestWorldModel::getCurAgent() != rp->getOwnID()) {
        return false;
    }
    if (alicaTests::TestWorldModel::getCurAgent() == 9) {
        return alicaTests::TestWorldModel::getOne()->isTransitionCondition3126176581533900616();
    } else if (alicaTests::TestWorldModel::getCurAgent() == 8) {
        return alicaTests::TestWorldModel::getTwo()->isTransitionCondition3126176581533900616();
    }
    return false;
    /*PROTECTED REGION END*/
}

/**
 * Transition: 1967586736681651770 (1967586736681651770)
 *   - Comment:
 *   - Source2Dest: 3235149896384117046 --> Finish
 *
 * Precondition: 2132248203469102498 (2132248203469102498)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in 3235149896384117046:
 *   - DynamicTaskTogether (1338298120374694644)
 */
bool PreCondition2132248203469102498::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1967586736681651770) ENABLED START*/
    std::cout << "The PreCondition 2132248203469102498 in Transition '1967586736681651770' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1602078208698393838) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
