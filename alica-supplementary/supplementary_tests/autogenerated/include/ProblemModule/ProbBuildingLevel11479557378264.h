#pragma once

#include "DomainCondition.h"
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
/*PROTECTED REGION ID(incl1479557378264) ENABLED START*/
// Add inlcudes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(meth1479557378264) ENABLED START*/
// Add other things here
/*PROTECTED REGION END*/
class UtilityFunction1479557378264 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
} /* namespace alica */
