#pragma once

#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>

#include <boost/dll/alias.hpp>

#include <alica_turtlesim/turtle_interfaces.hpp>

namespace turtlesim
{

class GoToCalculatedResult : public alica::BasicBehaviour
{
public:
    GoToCalculatedResult(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    void onTermination() override;
    static std::unique_ptr<GoToCalculatedResult> create(alica::BehaviourContext& context);

private:
    // std::shared_ptr<turtlesim::TurtleInterfaces> _turtle;
    // alica::Query _query;
    // std::vector<double> _results;
};
BOOST_DLL_ALIAS(turtlesim::GoToCalculatedResult::create, GoToCalculatedResult)

} /* namespace turtlesim */
