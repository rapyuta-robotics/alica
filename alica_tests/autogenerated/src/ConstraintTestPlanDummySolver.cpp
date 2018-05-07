#include "ConstraintTestPlanDummySolver.h"

#include <engine/AlicaEngine.h>
#include <engine/blackboard/BlackBoard.h>
#include <engine/constraintmodul/SolverVariable.h>
#include <engine/model/Variable.h>

#include <string>
#include <iostream>

using std::make_shared;
using std::shared_ptr;
using std::string;
using std::vector;

namespace alica {
namespace reasoner {
int ConstraintTestPlanDummySolver::existsSolutionCallCounter = 0;
int ConstraintTestPlanDummySolver::getSolutionCallCounter = 0;

ConstraintTestPlanDummySolver::ConstraintTestPlanDummySolver(AlicaEngine* ae)
        : ISolver(ae) {}

ConstraintTestPlanDummySolver::~ConstraintTestPlanDummySolver() {}

bool ConstraintTestPlanDummySolver::existsSolutionImpl(
        const VariableGrp& vars, const std::vector<shared_ptr<ProblemDescriptor>>& calls) {
    existsSolutionCallCounter++;
    // std::cout << "ConstraintTestPlanDummySolver::existsSolution was called " << existsSolutionCallCounter
    //		<< " times!" << std::endl;
    return false;
}

bool ConstraintTestPlanDummySolver::getSolutionImpl(const VariableGrp& vars,
        const std::vector<shared_ptr<ProblemDescriptor>>& calls, std::vector<BBIdent>& results) {
    BlackBoard& bb = getAlicaEngine()->editBlackBoard();
    for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
        const std::string& s = vars.at(i)->getName();
        results.push_back(bb.registerValue(s.c_str(), static_cast<int>(s.size())));
    }
    getSolutionCallCounter++;
    // std::cout << "ConstraintTestPlanDummySolver::getSolution was called " << getSolutionCallCounter << " times!"
    //		<< std::endl;
    return true;
}

int ConstraintTestPlanDummySolver::getExistsSolutionCallCounter() {
    return existsSolutionCallCounter;
}

int ConstraintTestPlanDummySolver::getGetSolutionCallCounter() {
    return getSolutionCallCounter;
}

shared_ptr<SolverVariable> ConstraintTestPlanDummySolver::createVariable(int64_t id) {
    return make_shared<SolverVariable>();
}
}  // namespace reasoner

} /* namespace alica */
