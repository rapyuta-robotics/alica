#pragma once

#include <boost/dll/alias.hpp>
#include <engine/constraintmodul/Query.h>
#include <vector>
#include <engine/BasicBehaviour.h>

namespace alica
{
class SolverTestBehaviour : public BasicBehaviour
{
public:
    SolverTestBehaviour(BehaviourContext& context);
    virtual ~SolverTestBehaviour();
    virtual void run();
    static std::unique_ptr<SolverTestBehaviour> create(alica::BehaviourContext& context);

    int getCallCounter();
    static std::vector<double> result;

protected:
    virtual void initialiseParameters();
    alica::Query _query;
    int callCounter;

private:
};
BOOST_DLL_ALIAS(alica::SolverTestBehaviour::create, SolverTestBehaviour)
} /* namespace alica */
