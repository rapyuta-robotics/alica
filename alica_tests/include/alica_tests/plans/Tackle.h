#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class Tackle : public BasicPlan
{
public:
    Tackle(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, Tackle)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TackleUtilityFunction)

} /* namespace alica */
