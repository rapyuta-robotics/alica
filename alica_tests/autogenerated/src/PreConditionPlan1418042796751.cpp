#include "PreConditionPlan1418042796751.h"
/*PROTECTED REGION ID(eph1418042796751) ENABLED START*/
// Add additional using directives here
#include "TestConstantValueSummand.h"
#include "TestWorldModel.h"
#include "engine/USummand.h"
#include <DistXContourTest.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PreConditionPlan
// Check of PreCondition - (Name): NewPreCondition, (ConditionString): Test , (Comment) :

/**
 * Available Vars:
 */
bool PreCondition1418042929966::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1418042929966) ENABLED START*/
    //--> "PreCondition:1418042929966  not implemented";
    //    	return true;
    return alicaTests::TestWorldModel::getOne()->isPreCondition1418042929966();
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042796753
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042796751::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042796751) ENABLED START*/
    shared_ptr<UtilityFunction> function = make_shared<UtilityFunction>(0.5, 0.1, plan);
    function->editUtilSummands().emplace_back(new TestConstantValueSummand(0.5, 1.0));
    return function;
    /*PROTECTED REGION END*/
}
} // namespace alica
