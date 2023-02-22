#pragma once

#include "engine/BasicPlan.h"
#include "engine/BasicUtilityFunction.h"
#include <boost/dll/alias.hpp>

namespace turtlesim
{

class MakeFormation : public alica::BasicPlan
{
public:
    MakeFormation(alica::PlanContext& context);
    static std::unique_ptr<MakeFormation> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::MakeFormation::create, MakeFormation)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MakeFormationUtilityFunction)

} /* namespace turtlesim */
