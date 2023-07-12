#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>

namespace turtlesim
{
class GoTo : public alica::BasicBehaviour
{
public:
    GoTo(alica::BehaviourContext& context);
    ~GoTo() override;
    virtual void run();
    static std::unique_ptr<GoTo> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();

private:
    alica::Query _query;
    std::vector<double> _results;
};
BOOST_DLL_ALIAS(turtlesim::GoTo::create, GoTo)
} /* namespace turtlesim */
