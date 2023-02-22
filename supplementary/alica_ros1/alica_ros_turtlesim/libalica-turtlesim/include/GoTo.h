#pragma once

#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>

#include <boost/dll/alias.hpp>

#include "turtle.hpp"

namespace turtlesim
{

class GoTo : public alica::BasicBehaviour
{
public:
    GoTo(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<GoTo> create(alica::BehaviourContext& context);

private:
    std::shared_ptr<turtlesim::ALICATurtle> _turtle;
    double _goal_x;
    double _goal_y;
};

BOOST_DLL_ALIAS(turtlesim::GoTo::create, GoTo)

} /* namespace turtlesim */
