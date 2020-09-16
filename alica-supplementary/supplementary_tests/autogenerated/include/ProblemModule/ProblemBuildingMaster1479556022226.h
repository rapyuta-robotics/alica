#pragma once

#include "DomainCondition.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1479556022226) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1479556022226) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class UtilityFunction1479556022226 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1479557592662 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
