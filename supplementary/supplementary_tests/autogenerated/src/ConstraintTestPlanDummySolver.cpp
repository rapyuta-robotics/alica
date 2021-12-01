#include "ConstraintTestPlanDummySolver.h"

#include <alica_solver_interface/SimpleContext.h>
#include <alica_solver_interface/SolverVariable.h>
#include <engine/AlicaEngine.h>
#include <engine/blackboard/BlackBoard.h>
#include <engine/model/Variable.h>

#include <iostream>
#include <string>

using std::make_shared;
using std::shared_ptr;
using std::string;
using std::vector;

namespace alica
{
namespace reasoner
{

int ConstraintTestPlanDummySolver::existsSolutionCallCounter = 0;
int ConstraintTestPlanDummySolver::getSolutionCallCounter = 0;

ConstraintTestPlanDummySolver::ConstraintTestPlanDummySolver(AlicaEngine* ae)
        : ISolver(ae)
{
}

ConstraintTestPlanDummySolver::~ConstraintTestPlanDummySolver() {}

bool ConstraintTestPlanDummySolver::existsSolutionImpl(SolverContext*, const std::vector<shared_ptr<ProblemDescriptor>>&)
{
    ++existsSolutionCallCounter;
    // std::cout << "ConstraintTestPlanDummySolver::existsSolution was called " << existsSolutionCallCounter
    //		<< " times!" << std::endl;
    return false;
}

bool ConstraintTestPlanDummySolver::getSolutionImpl(SolverContext* ctx, const std::vector<shared_ptr<ProblemDescriptor>>& calls, std::vector<BBIdent>& results)
{
    BlackBoard& bb = getAlicaEngine()->editBlackBoard();
    SimpleContext<SolverVariable>* tdc = static_cast<SimpleContext<SolverVariable>*>(ctx);
    for (const auto& var : tdc->getVariables()) {
        std::string s = std::to_string(var->getId());
        results.push_back(bb.registerValue(s.c_str(), static_cast<int>(s.size())));
    }
    ++getSolutionCallCounter;
    // std::cout << "ConstraintTestPlanDummySolver::getSolution was called " << getSolutionCallCounter << " times!"
    //		<< std::endl;
    return true;
}

int ConstraintTestPlanDummySolver::getExistsSolutionCallCounter()
{
    return existsSolutionCallCounter;
}

int ConstraintTestPlanDummySolver::getGetSolutionCallCounter()
{
    return getSolutionCallCounter;
}

SolverVariable* ConstraintTestPlanDummySolver::createVariable(int64_t id, SolverContext* ctx)
{
    return static_cast<SimpleContext<SolverVariable>*>(ctx)->createVariable(id);
}
std::unique_ptr<SolverContext> ConstraintTestPlanDummySolver::createSolverContext()
{
    return std::unique_ptr<SolverContext>(new SimpleContext<SolverVariable>());
}

} // namespace reasoner

} /* namespace alica */
