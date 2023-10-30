#pragma once

#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>

#include <boost/dll/alias.hpp>

#include <alica_turtlesim/turtle_interfaces.hpp>

namespace turtlesim
{

class GoTo : public alica::BasicBehaviour
{
public:
    GoTo(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    ~GoTo();
    static std::unique_ptr<GoTo> create(alica::BehaviourContext& context);

private:
    // std::shared_ptr<turtlesim::TurtleInterfaces> _turtle;
    double _goal_x = 0;
    double _goal_y = 0;
};

BOOST_DLL_ALIAS(turtlesim::GoTo::create, GoTo)

} /* namespace turtlesim */
