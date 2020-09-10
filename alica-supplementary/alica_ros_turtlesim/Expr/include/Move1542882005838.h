#pragma once

#include "DomainCondition.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1542882005838) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1542882005838) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class UtilityFunction1542882005838 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1543284793605 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
