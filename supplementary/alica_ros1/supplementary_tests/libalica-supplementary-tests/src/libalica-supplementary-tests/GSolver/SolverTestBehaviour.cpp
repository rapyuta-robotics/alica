#include <libalica-supplementary-tests/GSolver/SolverTestBehaviour.h>
#include <memory>

#include <constraintsolver/CGSolver.h>

namespace alica
{

std::vector<double> SolverTestBehaviour::result;

SolverTestBehaviour::SolverTestBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
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

std::unique_ptr<SolverTestBehaviour> SolverTestBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<SolverTestBehaviour>(context);
}

} /* namespace alica */
