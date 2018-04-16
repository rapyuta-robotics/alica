#include "alica/reasoner/DummySolver.h"
#include "alica/reasoner/DummyTerm.h"
#include "alica/reasoner/DummyVariable.h"

#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Variable.h>
#include <engine/blackboard/Blackboard.h>

namespace alica {

namespace reasoner {

DummySolver::DummySolver(AlicaEngine* ae)
        : ISolver(ae) {}

DummySolver::~DummySolver() {}

bool DummySolver::existsSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls) {
    return true;
}

bool DummySolver::getSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
        std::vector<BBIdent>& results) {

    //TODO: reformulate this without a temporary vector
    std::vector<std::shared_ptr<DummyVariable>> dummyVariables;
    dummyVariables.reserve(vars.size());
    for (auto variable : vars) {
        auto dummyVariable = std::dynamic_pointer_cast<alica::reasoner::DummyVariable>(variable->getSolverVar());
        if (!dummyVariable) {
            std::cerr << "DummySolver: Variable type does not match Solver type!" << std::endl;
        }
        dummyVariables.push_back(dummyVariable);
    }
    //TODO: reformulate this without a temporary map
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

    results.reserve(dummyVariables.size());
    BlackBoard& bb =  getAlicaEngine()->editBlackBoard();
    for (const auto& dummyVariable : dummyVariables) {
        const std::string& val = dummyVariableValueMap[dummyVariable->getID()];
        BBIDent bid = bb.registerValue(val.c_str(),val.size());
        results.push_back(bid);
    }

    return true;
}

std::shared_ptr<SolverVariable> DummySolver::createVariable(int64_t representingVariableID) {
    return std::make_shared<DummyVariable>(representingVariableID);
}

} /* namespace reasoner */

} /* namespace alica */
