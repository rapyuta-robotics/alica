#pragma once

#include "DomainCondition.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1402488893641) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1402488893641) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class UtilityFunction1402488893641 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1402488993122 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1402488991641 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1402489065962 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
class PreCondition1402489073613 : public DomainCondition
{
    bool evaluate(std::shared_ptr<RunningPlan> rp);
};
} /* namespace alica */
