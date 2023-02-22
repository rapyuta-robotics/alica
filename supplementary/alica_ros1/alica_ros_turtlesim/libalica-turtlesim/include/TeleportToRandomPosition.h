#pragma once

#include <engine/BasicBehaviour.h>
#include <engine/BasicUtilityFunction.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class TeleportToRandomPosition : public alica::BasicBehaviour
{
public:
    TeleportToRandomPosition(alica::BehaviourContext& context);
    void initialiseParameters() override;
    static std::unique_ptr<TeleportToRandomPosition> create(alica::BehaviourContext& context);
};

BOOST_DLL_ALIAS(turtlesim::TeleportToRandomPosition::create, TeleportToRandomPosition)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TeleportToRandomPositionUtilityFunction)

} /* namespace turtlesim */
