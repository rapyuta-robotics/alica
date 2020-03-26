#pragma once

#include "DomainCondition.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1524452759599) ENABLED START*/
// Add inlcudes here
extern bool vhStartCondition;
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1524452759599) ENABLED START*/
// Add other things here

/*PROTECTED REGION END*/
class UtilityFunction1524452759599 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class RunTimeCondition1524453470580 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1524453491764 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
