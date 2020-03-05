#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/RunningPlan.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/model/Condition.h"
#include "engine/model/VariableBinding.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"
#include "engine/model/Variable.h"
#include "engine/teammanager/Agent.h"
#include "engine/teammanager/TeamManager.h"
#include <engine/constraintmodul/ConditionStore.h>
#include <engine/constraintmodul/ISolver.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/constraintmodul/ProblemPart.h>
#include <engine/constraintmodul/Query.h>

#include <alica_solver_interface/SolverContext.h>
#include <alica_solver_interface/SolverTerm.h>
#include <alica_solver_interface/SolverVariable.h>

#include <assert.h>
#include <iostream>

namespace alica
{

Query::Query() {}

void Query::addStaticVariable(const Variable* v)
{
    _queriedStaticVariables.push_back(v);
}

void Query::addDomainVariable(essentials::IdentifierConstPtr agent, const std::string& ident, const AlicaEngine* ae)
{
    _queriedDomainVariables.push_back(ae->getTeamManager().getDomainVariable(agent, ident));
}

void Query::clearDomainVariables()
{
    _queriedDomainVariables.clear();
}

void Query::clearStaticVariables()
{
    _queriedStaticVariables.clear();
}

void Query::clearTemporaries()
{
    // Clear variable stuff and calls
    _uniqueVarStore.clear();
    _staticVars.clear();
    _domainVars.clear();
    _problemParts.clear();
}

/**
 * ONLY FOR TESTING!
 */
const UniqueVarStore& Query::getUniqueVariableStore() const
{
    return _uniqueVarStore;
}

void Query::fillBufferFromQuery()
{
    if (!_queriedStaticVariables.empty()) {
        _staticVars.editCurrent().insert(_staticVars.editCurrent().end(), _queriedStaticVariables.begin(), _queriedStaticVariables.end());
    }
    if (!_queriedDomainVariables.empty()) {
        _domainVars.editCurrent().insert(_domainVars.editCurrent().end(), _queriedDomainVariables.begin(), _queriedDomainVariables.end());
    }
    // add static variables into clean unique variable store
    _uniqueVarStore.initWith(_queriedStaticVariables);
}

bool Query::collectProblemStatement(ThreadSafePlanInterface pi, ISolverBase& solver, std::vector<std::shared_ptr<ProblemDescriptor>>& pds, int& domOffset)
{
    AlicaTime time;
#ifdef ALICA_DEBUG_ENABLED
    time = pi.getAlicaEngine()->getAlicaClock().now();
#endif

    clearTemporaries();
    // insert _queried_ variables of this query into the _relevant_ variables
    fillBufferFromQuery();

    ALICA_DEBUG_MSG("Query: Initial static buffer Size: " << _staticVars.getCurrent().size());
    ALICA_DEBUG_MSG("Query: Initial domain buffer Size: " << _domainVars.getCurrent().size());
    ALICA_DEBUG_MSG("Query: Starting Query with static Vars:" << std::endl << _uniqueVarStore);
    {
        ReadLockedPlanPointer rp = pi.getRunningPlan();
        // Goes recursive upwards in the plan tree and does three steps on each level.
        while (rp && (_staticVars.hasCurrentlyAny() || _domainVars.hasCurrentlyAny())) {
            ALICA_DEBUG_MSG("Query: Plantree-LVL of " << rp->getActivePlan()->getName() << std::endl << _uniqueVarStore);

            // 1. fill the query's static and domain variables, as well as its problem parts
            rp->getConstraintStore().acceptQuery(*this, rp.get());
            // next should be empty, current full

            ALICA_DEBUG_MSG("Query: Size of problemParts is " << _problemParts.size());

            // 2. process bindings for plantype
            if (rp->getPlanType() != nullptr) {
                for (const VariableBinding* p : rp->getPlanType()->getVariableBindings()) {
                    if (p->getSubPlan() == rp->getActivePlan() && _staticVars.hasCurrently(p->getSubVar())) {
                        _staticVars.editNext().push_back(p->getVar());
                        _uniqueVarStore.addVarTo(p->getSubVar(), p->getVar());
                    }
                }
                /**
                 * Overwrite relevantStaticVariables for having only the
                 * static variables of the current plan tree level for
                 * the next iteration. Each while-iteration goes one level up.
                 */
                _staticVars.flip();
            }

            // 3. process bindings for state
            RunningPlan* parent = rp->getParent();
            const State* parent_state = nullptr;
            if (parent && (parent_state = parent->getActiveState()) != nullptr) {
                _staticVars.editNext().clear();
                for (const VariableBinding* p : parent_state->getParametrisation()) {
                    if ((p->getSubPlan() == rp->getActivePlan() || p->getSubPlan() == rp->getPlanType()) && _staticVars.hasCurrently(p->getSubVar())) {
                        _staticVars.editNext().push_back(p->getVar());
                        _uniqueVarStore.addVarTo(p->getSubVar(), p->getVar());
                    }
                }
                /**
                 * Overwrite relevantStaticVariables for having only the
                 * static variables of the current plan tree level for
                 * the next iteration. Each while-iteration goes one level up.
                 */
                _staticVars.flip();
            }
            rp.moveUp();
        }
    }

    ALICA_DEBUG_MSG("Query: " << _uniqueVarStore);
    // now we have a vector<ProblemPart> in problemParts ready to be queried together with a store of unifications
    if (_problemParts.empty()) {
        ALICA_DEBUG_MSG("Query: Empty Query!");
        return false;
    }

    ALICA_DEBUG_MSG("Query: Size of problemParts is " << _problemParts.size());
    if (_context.get() == nullptr) {
        _context = solver.createSolverContext();
    } else {
        _context->clear();
    }
    // all solver variables will be created here:
    _uniqueVarStore.setupSolverVars(solver, _context.get(), _domainVars.getCurrent());
    for (ProblemPart& probPart : _problemParts) {
        pds.push_back(probPart.generateProblemDescriptor(solver, _uniqueVarStore, _context.get()));
    }

    _relevantVariables.clear();
    // write all static variables into the out-parameter "relevantVariables"
    _uniqueVarStore.getAllRep(_relevantVariables);

    // the index of the first domain variable after the static variables
    domOffset = _relevantVariables.size();

    // write all domain variables into the out-parameter "relevantVariables"
    if (!_domainVars.getCurrent().empty()) {
        _relevantVariables.insert(_relevantVariables.end(), _domainVars.getCurrent().begin(), _domainVars.getCurrent().end());
    }

    ALICA_DEBUG_MSG("Query: Number of relevant static variables: " << domOffset);
    ALICA_DEBUG_MSG("Query: Number of relevant domain variables: " << _domainVars.getCurrent().size());
    ALICA_DEBUG_MSG("Query: Total number of relevant variables: " << _relevantVariables.size());
    ALICA_DEBUG_MSG("Query: PrepTime: " << (pi.getAlicaEngine()->getAlicaClock().now() - time).inMicroseconds() << "us");

    return true;
}

void Query::addProblemPart(ProblemPart&& pp)
{
    const Condition* c = pp.getCondition();
    assert(std::find_if(_problemParts.begin(), _problemParts.end(), [c](const ProblemPart& opp) { return opp.getCondition() == c; }) == _problemParts.end());

    for (const AgentVariables& avars : pp.getAllVariables()) {
        for (const DomainVariable* domainvariable : avars.getVars()) {
            if (!_domainVars.has(domainvariable)) {
                ALICA_DEBUG_MSG("Query: Adding DomVar: " << *domainvariable);
                _domainVars.editCurrent().push_back(domainvariable);
            }
        }
    }

    for (const Variable* variable : c->getVariables()) {
        if (!_staticVars.has(variable)) {
            _staticVars.editCurrent().push_back(variable);
            _uniqueVarStore.addChecked(variable);
        }
    }

    _problemParts.push_back(std::move(pp));
}

} /* namespace alica */
