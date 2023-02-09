#include <memory>
#include <supplementary_tests/GSolver/SolverTestBehaviour.h>

#include <constraintsolver/CGSolver.h>

namespace alica
{

std::vector<double> SolverTestBehaviour::result;

SolverTestBehaviour::SolverTestBehaviour(BehaviourContext& context)
        : DomainBehaviour(context)
{

    callCounter = 0;
}
SolverTestBehaviour::~SolverTestBehaviour() {}
void SolverTestBehaviour::run()
{
    callCounter++;
    _query.getSolution<reasoner::CGSolver, double>(getPlanContext(), result);
}
void SolverTestBehaviour::initialiseParameters()
{
    _query.clearStaticVariables();
    _query.addStaticVariable(getVariable("X"));
    _query.addStaticVariable(getVariable("Y"));
}

int SolverTestBehaviour::getCallCounter()
{
    return callCounter;
}

} /* namespace alica */
