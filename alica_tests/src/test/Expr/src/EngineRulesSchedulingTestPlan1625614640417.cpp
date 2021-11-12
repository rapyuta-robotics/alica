#include "EngineRulesSchedulingTestPlan1625614640417.h"
/*PROTECTED REGION ID(eph1625614640417) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  EngineRulesSchedulingTestPlan (1625614640417)
//
// Tasks:
//   - EngineRulesSchedulingTestEntry (1625610762033) (Entrypoint: 1625614705483)//   - EngineRulesSchedulingTestSecondEntry (1625610785404) (Entrypoint:
//   1625614710816)
//
// States:
//   - Default Name (1625614697742)
//   - StartEngineRulesSchedulingTest (1625614714499)
//   - Default Name (1625614719367)
//   - FailureState (1625776883489)
EngineRulesSchedulingTestPlan1625614640417::EngineRulesSchedulingTestPlan1625614640417()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1625614640417) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
EngineRulesSchedulingTestPlan1625614640417::~EngineRulesSchedulingTestPlan1625614640417()
{
    /*PROTECTED REGION ID(dcon1625614640417) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: EngineRulesSchedulingTestEntry  -> EntryPoint-ID: 1625614705483
 * Task: EngineRulesSchedulingTestSecondEntry  -> EntryPoint-ID: 1625614710816
 */
std::shared_ptr<UtilityFunction> UtilityFunction1625614640417::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1625614640417) ENABLED START*/
    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromDefault NameTo Default Name (1625614729978)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: StartEngineRulesSchedulingTest --> Default Name
 *
 * Precondition: 1625614729981 (1625614729981)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in StartEngineRulesSchedulingTest:
 *   - EmptyBehaviour (1625610857563)
 */
bool PreCondition1625614729981::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1625614729978) ENABLED START*/
    return alicaTests::TestWorldModel::getOne()->isTransitionCondition1625614729978();
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromStartEngineRulesSchedulingTestTo Default Name (1625776897471)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: StartEngineRulesSchedulingTest --> FailureState
 *
 * Precondition: 1625776897472 (1625776897472)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in StartEngineRulesSchedulingTest:
 *   - EmptyBehaviour (1625610857563)
 */
bool PreCondition1625776897472::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1625776897471) ENABLED START*/
    if (alicaTests::TestWorldModel::getOne()->isTransitionCondition1625776897472()) {
        alicaTests::TestWorldModel::getOne()->setTransitionCondition1625776897472(false);
        return true;
    }
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1625614640417) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
