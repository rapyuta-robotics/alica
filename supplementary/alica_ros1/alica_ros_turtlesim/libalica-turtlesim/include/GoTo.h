#pragma once

#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class GoTo : public alica::BasicBehaviour
{
public:
    GoTo(alica::BehaviourContext& context);
    void run() override;
    static std::unique_ptr<GoTo> create(alica::BehaviourContext& context);
    void initialiseParameters() override;

private:
    alica::Query _query;
    std::vector<double> _results;
};
BOOST_DLL_ALIAS(turtlesim::GoTo::create, GoTo)

} /* namespace turtlesim */
