#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class SpawnTurtle : public alica::BasicBehaviour
{
public:
    SpawnTurtle(alica::BehaviourContext& context);
    void initialiseParameters() override;
    static std::unique_ptr<SpawnTurtle> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(turtlesim::SpawnTurtle::create, SpawnTurtle)

} /* namespace turtlesim */
