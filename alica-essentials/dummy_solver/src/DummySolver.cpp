#include "alica/reasoner/DummySolver.h"
#include "alica/reasoner/DummyTerm.h"
#include "alica/reasoner/DummyVariable.h"

#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Variable.h>

namespace alica {

namespace reasoner {

DummySolver::DummySolver(AlicaEngine* ae) : alica::ISolver(ae) {}

DummySolver::~DummySolver() {}

bool DummySolver::existsSolution(std::vector<Variable*>& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls) {
    return true;
}

bool DummySolver::getSolution(std::vector<Variable*>& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
        std::vector<void*>& results) {
    std::vector<std::shared_ptr<DummyVariable>> dummyVariables;
    dummyVariables.reserve(vars.size());
    for (auto variable : vars) {
        auto dummyVariable = std::dynamic_pointer_cast<alica::reasoner::DummyVariable>(variable->getSolverVar());
        if (!dummyVariable) {
            std::cerr << "DummySolver: Variable type does not match Solver type!" << std::endl;
        }
        dummyVariables.push_back(dummyVariable);
    }

    std::map<long, std::string> dummyVariableValueMap;

    for (auto& c : calls) {
        auto constraintTerm = std::dynamic_pointer_cast<DummyTerm>(c->getConstraint());
        if (!constraintTerm) {
            std::cerr << "DummySolver: Constraint type not compatible with selected solver!" << std::endl;
            return false;
        }
        for (auto dummyVariable : dummyVariables) {
            std::string value = constraintTerm->getValue(dummyVariable);
            auto mapEntry = dummyVariableValueMap.find(dummyVariable->getID());
            if (mapEntry == dummyVariableValueMap.end()) {
                // insert new value
                dummyVariableValueMap.emplace(dummyVariable->getID(), value);
                continue;
            }

            // check consistence of values
            if (mapEntry->second != value) {
                return false;
            }
        }
    }

    // TODO: set results
    results.reserve(dummyVariables.size());
    for (auto dummyVariable : dummyVariables) {
        results.push_back(new std::string(dummyVariableValueMap[dummyVariable->getID()]));
    }

    return true;
}

std::shared_ptr<SolverVariable> DummySolver::createVariable(long representingVariableID) {
    return std::make_shared<DummyVariable>(representingVariableID);
}

} /* namespace reasoner */

} /* namespace alica */
