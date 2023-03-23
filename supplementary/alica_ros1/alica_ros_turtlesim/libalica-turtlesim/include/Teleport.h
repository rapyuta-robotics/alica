#pragma once

#include "turtle_interfaces.hpp"
#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class Teleport : public alica::BasicBehaviour
{
private:
    std::shared_ptr<turtlesim::TurtleInterfaces> _turtle;
    double _x;
    double _y;

public:
    Teleport(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<Teleport> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Teleport::create, Teleport)

} /* namespace turtlesim */
