#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class TeleportToRandomPosition : public alica::BasicPlan
{
public:
    TeleportToRandomPosition(alica::PlanContext& context);
    void onInit() override;
    static std::unique_ptr<TeleportToRandomPosition> create(alica::PlanContext& context);
};

BOOST_DLL_ALIAS(turtlesim::TeleportToRandomPosition::create, TeleportToRandomPosition)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TeleportToRandomPositionUtilityFunction)

} /* namespace turtlesim */
