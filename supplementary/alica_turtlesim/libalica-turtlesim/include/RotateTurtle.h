#pragma once

#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>

#include <boost/dll/alias.hpp>

#include <alica_turtlesim/turtle_interfaces.hpp>

namespace turtlesim
{

class RotateTurtle : public alica::BasicBehaviour
{
public:
    RotateTurtle(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<RotateTurtle> create(alica::BehaviourContext& context);

private:
    std::shared_ptr<turtlesim::TurtleInterfaces> _turtle;
};

BOOST_DLL_ALIAS(turtlesim::RotateTurtle::create, RotateTurtle)

} /* namespace turtlesim */
