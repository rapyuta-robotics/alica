#include "alica/reasoner/DummySolver.h"
#include "alica/reasoner/DummyTerm.h"
#include "alica/reasoner/DummyVariable.h"

#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Variable.h>

namespace alica
{

namespace reasoner
{

DummySolver::DummySolver(AlicaEngine *ae)
    : alica::ISolver(ae)
{
}

DummySolver::~DummySolver()
{
}

bool DummySolver::existsSolution(std::vector<Variable *> &vars, std::vector<std::shared_ptr<ProblemDescriptor>> &calls)
{
    return true;
}

bool DummySolver::getSolution(std::vector<Variable *> &vars, std::vector<std::shared_ptr<ProblemDescriptor>> &calls,
                              std::vector<void *> &results)
{
	std::vector<std::shared_ptr<DummyVariable>> variables;
	variables.resize(vars.size());
    for (auto variable : vars)
    {
        auto dummyVariable = std::dynamic_pointer_cast<alica::reasoner::DummyVariable>(variable->getSolverVar());
        if (!dummyVariable)
        {
            std::cerr << "DummySolver: Variable type does not match Solver type!" << std::endl;
        }
        variables.push_back(dummyVariable);
    }

    for (auto &c : calls)
    {
        auto constraintTerm = std::dynamic_pointer_cast<DummyTerm>(c->getConstraint());
        if (!constraintTerm)
        {
            std::cerr << "DummySolver: Constraint type not compatible with selected solver!" << std::endl;
            return false;
        }

        constraintTerm->getValue(dummyVariable);
    }
    // TODO: compare set values of variables

    // TODO: set results
}

return true;
}

std::shared_ptr<SolverVariable> DummySolver::createVariable(long representingVariableID)
{
    return std::make_shared<DummyVariable>(representingVariableID);
}

} /* namespace reasoner */

} /* namespace alica */
