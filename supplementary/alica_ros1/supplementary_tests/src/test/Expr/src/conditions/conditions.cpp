#include <supplementary_tests/conditions/conditions.h>

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

/*PROTECTED REGION ID(conditionSource) ENABLED START*/
// Add additional options here
bool vhStartCondition = false;
/*PROTECTED REGION END*/

namespace alica
{
bool conditionVariableHandlingStart295816226925111421(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels)
{
    /*PROTECTED REGION ID(condition295816226925111421) ENABLED START*/
    return vhStartCondition;
    /*PROTECTED REGION END*/
}
bool conditionDefaultCondition2011598442725310989(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels)
{
    /*PROTECTED REGION ID(condition2011598442725310989) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
} /* namespace alica */
