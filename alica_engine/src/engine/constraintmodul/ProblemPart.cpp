#include "engine/constraintmodul/ProblemPart.h"

#include "engine/RunningPlan.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/model/Condition.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/Quantifier.h"

namespace alica {

ProblemPart::ProblemPart(const Condition* con, shared_ptr<RunningPlan> rp)
        : _condition(con)
        , _runningPlan(rp)
        , _descriptor(std::make_shared<ProblemDescriptor>()) {
    for (const Quantifier* quantifier : con->getQuantifiers()) {
        quantifier->addDomainVariables(rp, _vars);
    }
}

ProblemPart::ProblemPart(ProblemPart&& o)
        : _vars(std::move(o._vars))
        , _condition(o._condition)
        , _runningPlan(std::move(o._runningPlan))
        , _descriptor(std::move(o._descriptor)) {}

ProblemPart& ProblemPart::operator=(ProblemPart&& o) {
    _vars = std::move(o._vars);
    _condition = o._condition;
    _runningPlan = std::move(o._runningPlan);
    _descriptor = std::move(o._descriptor);
    return *this;
}

/**
 * Checks whether the given variable is one of the domain variables.
 */
bool ProblemPart::hasVariable(const DomainVariable* v) const {
    for (const AgentVariables& avars : _vars) {
        for (const DomainVariable* variable : avars.getVars()) {
            if (variable == v) {
                return true;
            }
        }
    }
    return false;
}
std::shared_ptr<ProblemDescriptor> ProblemPart::generateProblemDescriptor(
        ISolverBase* solver, UniqueVarStore& uvs) const {
    _descriptor.clear();
    const VariableSet& staticCondVariables = getCondition()->getVariables();

    // create a vector of solver variables from the static condition variables

    int dim = static_cast<int>(staticCondVariables.size());
    _descriptor->_staticVars.reserve(dim);

    for (const Variable* variable : staticCondVariables) {
        const Variable* representingVariable = uvs.getRep(variable);
        if (representingVariable->getSolverVar() == nullptr) {
            representingVariable->setSolverVar(solver->createVariable(representingVariable->getId()));
        }
        _descriptor->_staticVars.push_back(representingVariable->getSolverVar());
    }

    // create a vector of solver variables from the domain variables of the currently iterated problem part
    _descriptor->_domainVars.reserve(_vars.size());
    for (const AgentVariables& avars : _vars) {
        AgentSolverVariables asolverVars(avars.getId());
        int curDim = static_cast<int>(avars.getVars().size());
        asolverVars.editVars().reserve(curDim);
        for (const DomainVariable* dv : avars.getVars()) {
            if (dv->getSolverVar() == nullptr) {
                dv->setSolver(solver->createVariable(dv->getId()));
            }
            asolverVars.editVars().emplace_back(dv->getSolverVar());
        }
        _descriptor->domainVars.push_back(std::move(asolverVars));
        dim += curDim;
    }
    // this insert the actual problem description
    _descriptor->_dim = dim;
    _descriptor->prepForUsage();

    getCondition()->getConstraint(_descriptor, _runningPlan);
    return _descriptor;
}
} /* namespace alica */
