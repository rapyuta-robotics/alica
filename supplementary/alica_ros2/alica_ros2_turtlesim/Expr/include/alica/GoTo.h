#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>
#include <engine/constraintmodul/Query.h>

namespace alica
{
class GoTo : public BasicBehaviour
{
public:
    GoTo(BehaviourContext& context);
    virtual ~GoTo();
    virtual void run();
    static std::unique_ptr<GoTo> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();

private:
    alica::Query _query;
    std::vector<double> _results;
};
BOOST_DLL_ALIAS(alica::GoTo::create, GoTo)
} /* namespace alica */
