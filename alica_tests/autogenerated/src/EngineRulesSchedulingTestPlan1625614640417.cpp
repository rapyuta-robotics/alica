#include "EngineRulesSchedulingTestPlan1625614640417.h"
/*PROTECTED REGION ID(eph1625614640417) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:EngineRulesSchedulingTestPlan1625614640417
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
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1625614729981, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - EngineRulesSchedulingTestEntry (1625610762033) (Entrypoint: 1625614705483)*   - EngineRulesSchedulingTestSecondEntry (1625610785404) (Entrypoint:
 * 1625614710816)
 *
 * States in plan:
 *   - Default Name (1625614697742)
 *   - StartEngineRulesSchedulingTest (1625614714499)
 *   - Default Name (1625614719367)
 *
 * Variables of precondition:
 */
bool PreCondition1625614729981::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1625614729978) ENABLED START*/
    std::cout << "The PreCondition 1625614729981 in Transition 'FromDefault NameTo Default Name' is not implement yet!" << std::endl;
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1625614640417) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
