#pragma once

#include "engine/BasicBehaviour.h"
#include "engine/constraintmodul/Query.h"
#include <boost/dll/alias.hpp>

namespace alica
{
class GoTo : public BasicBehaviour
{
public:
    GoTo(BehaviourContext& context);
    virtual ~GoTo();
    virtual void run();
    // Factory method
    static std::unique_ptr<GoTo> create(BehaviourContext& context) { return std::unique_ptr<GoTo>(new GoTo(context)); }

protected:
    virtual void initialiseParameters();

private:
    alica::Query _query;
    std::vector<double> _results;
};
BOOST_DLL_ALIAS(alica::GoTo::create, GoTo)
} /* namespace alica */
