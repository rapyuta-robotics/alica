#include <supplementary_tests/conditions/conditions.h>

#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

bool vhStartCondition = false;

namespace alica
{
bool conditionMISSING_NAME1479557592662(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition1479557592662) ENABLED START*/
    return vhStartCondition;
    /*PROTECTED REGION END*/
}
} /* namespace alica */
