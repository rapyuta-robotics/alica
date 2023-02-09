#pragma once

#include <engine/constraintmodul/Query.h>
#include <supplementary_tests/DomainBehaviour.h>
#include <vector>

namespace alica
{
class SolverTestBehaviour : public DomainBehaviour
{
public:
    SolverTestBehaviour(BehaviourContext& context);
    virtual ~SolverTestBehaviour();
    virtual void run();
    int getCallCounter();
    static std::vector<double> result;

protected:
    virtual void initialiseParameters();
    alica::Query _query;
    int callCounter;

private:
};
} /* namespace alica */
