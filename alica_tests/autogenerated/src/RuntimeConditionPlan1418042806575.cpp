#include "RuntimeConditionPlan1418042806575.h"
/*PROTECTED REGION ID(eph1418042806575) ENABLED START*/
// Add additional using directives here
#include "TestConstantValueSummand.h"
#include "TestWorldModel.h"
#include "engine/USummand.h"
#include <DistXContourTest.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:RuntimeConditionPlan
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition1418042967134::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1418042967134) ENABLED START*/
    return alicaTests::TestWorldModel::getOne()->isRuntimeCondition1418042967134();
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042806577
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042806575::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042806575) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
