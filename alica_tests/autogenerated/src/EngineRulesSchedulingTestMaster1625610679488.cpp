#include "EngineRulesSchedulingTestMaster1625610679488.h"
/*PROTECTED REGION ID(eph1625610679488) ENABLED START*/
// Add additional options here
#include <TestWorldModel.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:EngineRulesSchedulingTestMaster1625610679488
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
 * Outgoing transition:
 *   - Name: 1625783867495, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - EmptyBehaviour (1625610857563)
 *
 * Tasks in plan:
 *   - EngineRulesTest (1625614670867) (Entrypoint: 1625614674465)
 *
 * States in plan:
 *   - GoIntoSubPlan (1625614677498)
 *   - EntryState (1625783824098)
 *   - FailureState (1625783835198)
 *
 * Variables of precondition:
 */
bool PreCondition1625783867495::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1625783867494) ENABLED START*/
    return alicaTests::TestWorldModel::getOne()->isTransitionCondition1625783867495();
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1625783869825, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - EmptyBehaviour (1625610857563)
 *
 * Tasks in plan:
 *   - EngineRulesTest (1625614670867) (Entrypoint: 1625614674465)
 *
 * States in plan:
 *   - GoIntoSubPlan (1625614677498)
 *   - EntryState (1625783824098)
 *   - FailureState (1625783835198)
 *
 * Variables of precondition:
 */
bool PreCondition1625783869825::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1625783869824) ENABLED START*/
    if (alicaTests::TestWorldModel::getOne()->isTransitionCondition1625783869825()) {
        alicaTests::TestWorldModel::getOne()->setTransitionCondition1625783869825(false);
        return true;
    }
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1625610679488) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
