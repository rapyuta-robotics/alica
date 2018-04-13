#include "alica/reasoner/DummySolver.h"
#include "alica/reasoner/DummyTerm.h"
#include "alica/reasoner/DummyVariable.h"

#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Variable.h>

namespace alica {

namespace reasoner {

DummySolver::DummySolver(AlicaEngine* ae)
        : ISolver(ae) {}

DummySolver::~DummySolver() {}

bool DummySolver::existsSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls) {
    return true;
}

bool DummySolver::getSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
        std::vector<int64_t>& results) {
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

    // TODO: register strings in blackboard
    results.reserve(dummyVariables.size());
    for (const auto& dummyVariable : dummyVariables) {
        results.push_back(dummyVariable->getID());
        //results.push_back(new std::string(dummyVariableValueMap[dummyVariable->getID()]));
    }

    return true;
}

std::shared_ptr<SolverVariable> DummySolver::createVariable(int64_t representingVariableID) {
    return std::make_shared<DummyVariable>(representingVariableID);
}

} /* namespace reasoner */

} /* namespace alica */
