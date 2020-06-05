#pragma once

#include "DomainCondition.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1414403396328) ENABLED START*/
#include <memory>
using namespace std;
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1414403396328) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class UtilityFunction1414403396328 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1414403842622 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
