#include "EngineRulesSchedulingTestMaster1625610679488.h"
/*PROTECTED REGION ID(eph1625610679488) ENABLED START*/
// Add additional options here
#include <alica_tests/TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  EngineRulesSchedulingTestMaster (1625610679488)
//
// Tasks:
//   - EngineRulesTest (1625614670867) (Entrypoint: 1625614674465)
//
// States:
//   - GoIntoSubPlan (1625614677498)
//   - EntryState (1625783824098)
//   - FailureState (1625783835198)
//   - Default Name (1626848011700)
EngineRulesSchedulingTestMaster1625610679488::EngineRulesSchedulingTestMaster1625610679488()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1625610679488) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
EngineRulesSchedulingTestMaster1625610679488::~EngineRulesSchedulingTestMaster1625610679488()
{
    /*PROTECTED REGION ID(dcon1625610679488) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: EngineRulesTest  -> EntryPoint-ID: 1625614674465
 */
std::shared_ptr<UtilityFunction> UtilityFunction1625610679488::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1625610679488) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromGoIntoSubPlanTo Default Name (1626848015857)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: GoIntoSubPlan --> Default Name
 *
 * Precondition: 1626848015861 (1626848015861)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in GoIntoSubPlan:
 *   - EngineRulesSchedulingTestPlan (1625614640417)
 */
bool PreCondition1626848015861::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1626848015857) ENABLED START*/
    auto* wm = dynamic_cast<alicaTests::TestWorldModel*>(rp->getWorldModel());
    return wm->isTransitionCondition1626848015861();
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromEntryStateTo GoIntoSubPlan (1625783867494)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: EntryState --> GoIntoSubPlan
 *
 * Precondition: 1625783867495 (1625783867495)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 *   - EmptyBehaviour (1625610857563)
 */
bool PreCondition1625783867495::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1625783867494) ENABLED START*/
    auto* wm = dynamic_cast<alicaTests::TestWorldModel*>(rp->getWorldModel());
    return wm->isTransitionCondition1625783867495();
    /*PROTECTED REGION END*/
}
/**
 * Transition: FromEntryStateTo FailureState (1625783869824)
 *   - Comment: MISSING_COMMENT
 *   - Source2Dest: EntryState --> FailureState
 *
 * Precondition: 1625783869825 (1625783869825)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in EntryState:
 *   - EmptyBehaviour (1625610857563)
 */
bool PreCondition1625783869825::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1625783869824) ENABLED START*/
    auto* wm = dynamic_cast<alicaTests::TestWorldModel*>(rp->getWorldModel());
    if (wm->isTransitionCondition1625783869825()) {
        wm->setTransitionCondition1625783869825(false);
        return true;
    }
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1625610679488) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
