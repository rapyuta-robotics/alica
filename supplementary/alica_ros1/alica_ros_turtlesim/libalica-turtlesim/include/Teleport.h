#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class Teleport : public alica::BasicBehaviour
{
public:
    Teleport(alica::BehaviourContext& context);
    void initialiseParameters() override;
    static std::unique_ptr<Teleport> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Teleport::create, Teleport)

} /* namespace turtlesim */
