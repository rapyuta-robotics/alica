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

/*PROTECTED REGION ID(methods1625610679488) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
