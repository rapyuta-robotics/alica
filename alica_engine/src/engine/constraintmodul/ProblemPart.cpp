#include "engine/constraintmodul/ProblemPart.h"

#include "engine/RunningPlan.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/model/Condition.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/Quantifier.h"

#include <alica_solver_interface/SolverContext.h>

#include <assert.h>

namespace alica
{

ProblemPart::ProblemPart(const Condition* con, std::shared_ptr<const RunningPlan> rp)
    : _condition(con)
    , _runningPlan(rp)
    , _descriptor()
{
    for (const Quantifier* quantifier : con->getQuantifiers()) {
        quantifier->addDomainVariables(rp, _vars);
    }
}

ProblemPart::ProblemPart(ProblemPart&& o)
    : _vars(std::move(o._vars))
    , _condition(o._condition)
    , _runningPlan(std::move(o._runningPlan))
    , _descriptor(std::move(o._descriptor))
{
}

ProblemPart& ProblemPart::operator=(ProblemPart&& o)
{
    _vars = std::move(o._vars);
    _condition = o._condition;
    _runningPlan = std::move(o._runningPlan);
    _descriptor = std::move(o._descriptor);
    return *this;
}

/**
 * Checks whether the given variable is one of the domain variables.
 */
bool ProblemPart::hasVariable(const DomainVariable* v) const
{
    for (const AgentVariables& avars : _vars) {
        for (const DomainVariable* variable : avars.getVars()) {
            if (variable == v) {
                return true;
            }
        }
    }
    return false;
}
std::shared_ptr<ProblemDescriptor> ProblemPart::generateProblemDescriptor(ISolverBase* solver, const UniqueVarStore& uvs, SolverContext* ctx)
{
    if (_descriptor == nullptr) {
        _descriptor = std::make_shared<ProblemDescriptor>(ctx);
    }
    assert(_descriptor->getContext() == ctx);
    _descriptor->clear();
    const VariableGrp& staticCondVariables = getCondition()->getVariables();

    // create a vector of solver variables from the static condition variables

    int dim = static_cast<int>(staticCondVariables.size());
    _descriptor->_staticVars.reserve(dim);

    for (const Variable* variable : staticCondVariables) {
        _descriptor->_staticVars.push_back(uvs.getSolverVariable(variable));
    }

    // create a vector of solver variables from the domain variables of the currently iterated problem part
    _descriptor->_domainVars.reserve(_vars.size());
    for (const AgentVariables& avars : _vars) {
        AgentSolverVariables asolverVars(avars.getId());
        int curDim = static_cast<int>(avars.getVars().size());
        asolverVars.editVars().reserve(curDim);
        for (const DomainVariable* dv : avars.getVars()) {
            asolverVars.editVars().emplace_back(uvs.getSolverVariable(dv, solver, ctx));
        }
        _descriptor->_domainVars.push_back(std::move(asolverVars));
        dim += curDim;
    }
    // this insert the actual problem description
    _descriptor->_dim = dim;
    _descriptor->prepForUsage();

    getCondition()->getConstraint(_descriptor, _runningPlan);
    return _descriptor;
}
} /* namespace alica */
