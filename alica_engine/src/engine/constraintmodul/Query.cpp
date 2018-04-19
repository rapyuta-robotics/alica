#include <engine/constraintmodul/ConditionStore.h>
#include <engine/constraintmodul/ISolver.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/constraintmodul/ProblemPart.h>
#include <engine/constraintmodul/Query.h>
#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/AlicaClock.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/teammanager/Agent.h"
#include "engine/RunningPlan.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"
#include "engine/model/Condition.h"
#include "engine/model/State.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/PlanType.h"
#include "engine/model/Variable.h"

#include <iostream>
//#define Q_DEBUG

namespace alica {
Query::Query() {}

void Query::addStaticVariable(const Variable* v) {
    queriedStaticVariables.push_back(v);
}

void Query::addDomainVariable(const supplementary::AgentID* robot, const std::string& ident, AlicaEngine* ae) {
    queriedDomainVariables.push_back(ae->getTeamManager()->getDomainVariable(robot, ident));
}

void Query::clearDomainVariables() {
    queriedDomainVariables.clear();
}

void Query::clearStaticVariables() {
    queriedStaticVariables.clear();
}


void Query::clearTemporaries() {
    // Clear variable stuff and calls
    _uniqueVarStore.clear();
    _staticVars.clear();
    _domainVars.clear();
    _problemParts.clear();
}
void Query::fillBufferFromQuery() {
    if (!_queriedStaticVariables.empty()) {
        _staticVars.editCurrent().insert(vars.current.staticVars.end(),_queriedStaticVariables.begin(),_queriedStaticVariables.end());
    }
    if (!queriedDomainVariables.empty()) {
        _domainVars.editCurrent().staticVars.insert(vars.current.staticVars.end(),_queriedDomainVariables.begin(),_queriedDomainVariables.end());
    }
    // add static variables into clean unique variable store
    _store.initWith(_queriedStaticVariables);

}

bool Query::collectProblemStatement(std::shared_ptr<RunningPlan> rp, ISolverBase* solver,
        std::vector<std::shared_ptr<ProblemDescriptor>>& pds, VariableSet& relevantVariables, int& domOffset) {
#ifdef Q_DEBUG
    AlicaTime time = rp->getAlicaEngine()->getAlicaClock()->now();
#endif

    clearTemporaries();
    // insert _queried_ variables of this query into the _relevant_ variables
    fillBufferFromQuery();

#ifdef Q_DEBUG
    std::cout << "Query: Initial static buffer Size: " << _staticVars.getCurrent().size() << std::endl;
    std::cout << "Query: Initial domain buffer Size: " << _domainVars.getCurrent().size() << std::endl;
    std::cout << "Query: Starting Query with static Vars:" << std::endl << _uniqueVarStore << std::endl;
#endif
    // Goes recursive upwards in the plan tree and does three steps on each level.
    while (rp != nullptr && (_staticVars.hasCurrentlyAny() || _domainVars.hasCurrentlyAny()) {
#ifdef Q_DEBUG
        cout << "Query: Plantree-LVL of " << rp->getPlan()->getName() << std::endl << _uniqueVarStore << std::endl;
#endif

        // 1. fill the query's static and domain variables, as well as its problem parts
        rp->getConstraintStore().acceptQuery(*this, rp);
        //next should be empty, current full
#ifdef Q_DEBUG
        std::cout << "Query: Size of problemParts is " << _problemParts.size() << std::endl;
#endif
        // 2. process bindings for plantype
        if (rp->getPlanType() != nullptr) {
            for (const Parametrisation* p : rp->getPlanType()->getParametrisation()) {
                if (p->getSubPlan() == rp->getPlan() && _staticVars.hasCurrently(p->getSubVar()) {

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
        shared_ptr<RunningPlan> parent;
        if (!rp->getParent().expired()) {
            parent = rp->getParent().lock();
        }
        if (parent && parent->getActiveState() != nullptr) {
            _staticVars.editNext().clear();
            for (const Parametrisation* p : parent->getActiveState()->getParametrisation()) {
                if ((p->getSubPlan() == rp->getPlan() || p->getSubPlan() == rp->getPlanType()) &&
                        _staticVars.hasCurrently(p->getSubVar())) {
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
        rp = parent;
    }
#ifdef Q_DEBUG
    cout << "Query: " << _uniqueVarStore << endl;
#endif
    // now we have a vector<ProblemPart> in problemParts ready to be queried together with a store of unifications
    if (_problemParts.empty()) {
#ifdef Q_DEBUG
        cout << "Query: Empty Query!" << endl;
#endif
        return false;
    }

#ifdef Q_DEBUG
    std::cout << "Query: Size of problemParts is " << _problemParts.size() << std::endl;
#endif

    for (const ProblemPart& probPart : _problemParts) {
        pds.push_back(probPart.generateDescriptor());
       
       
    }

    // write all static variables into the out-parameter "relevantVariables"
    relevantVariables = _uniqueVarStore.getAllRep();

    // the index of the first domain variable after the static variables
    domOffset = relevantVariables.size();

    // write all domain variables into the out-parameter "relevantVariables"
    if (!relevantDomainVariables.empty()) {
        relevantVariables.insert(
                relevantVariables.end(), _domainVarBuffer.getCurrent().begin(), _domainVarBuffer.getCurrent().end());
    }

#ifdef Q_DEBUG
    std::cout << "Query: Number of relevant static variables: " << domOffset << std::endl;
    std::cout << "Query: Number of relevant domain variables: " << _domainVarBuffer.getCurrent().size() << std::endl;
    std::cout << "Query: Total number of relevant variables: " << relevantVariables.size() << std::endl;
    cout << "Query: PrepTime: " << (rp->getAlicaEngine()->getAlicaClock()->now() - time).inMicroSeconds() <<"us"<< endl;
#endif
    return true;
}

void Query::addProblemPart(ProblemPart&& pp) {
    const Condition* c = pp->getCondition();
    assert(
        std::find_if(_problemParts.begin(),_problemParts.end(),[c](const ProblemPart& opp){return opp->getCondition()==c;}) == _problemPars.end()
    );

    for (const AgentVariables& avars : pp.getAllVariables()) {
        for (const Variable* Domainvariable : avars.getVars()) {
            if (!_domainVars.has(variable)) {
#ifdef Q_DEBUG
                    std::cout << "Query: Adding DomVar: " << *variable << std::endl;
#endif
                _domainVars.editCurrent().push_back(variable);
            }
        }
    }

    for (const Variable* variable : c->getVariables()) {
        if (!_staticVars.has(variable)) {
            _staticVars.editCurrent().push_back(variable);
        }
    }

    _problemParts.push_back(std::move(pp));
}

/// Part for the implementation of the internal class UniqueVarStore

UniqueVarStore::UniqueVarStore() {}

void UniqueVarStore::clear() {
    _store.clear();
}
void initWith(const VariableSet& vs) {
    _store.resize(vs.size());
    int i =0;
    for(const Variable* v : vs) [
        _store[i].push_back(v);
        assert(_store[i].size()==1);
        //TODO: make sure there is always one element, so we don't need to push back
    ]
}
/**
 * Initializes a list with the given variable and put that list into the internal store.
 */
void UniqueVarStore::add(const Variable* v) {
    VariableSet l;
    l.push_back(v);
    _store.push_back(std::move(l));
}

/**
 * Add the variable "toAdd" to the front of the list of variables that contains the variable "representing".
 * If such a list does not exist, a new list will be created.
 */
void UniqueVarStore::addVarTo(const Variable* representing, const Variable* toAdd) {
    for (auto& variables : store) {
        for (auto& variable : variables) {
            if (representing == variable) {
                variables.insert(variables.begin(), toAdd);
                return;
            }
        }
    }
    VariableSet nl;
    nl.insert(nl.begin(), representing);
    nl.insert(nl.begin(), toAdd);
    store.push_back(std::move(nl));
}

VariableSet UniqueVarStore::getAllRep() const {
    VariableSet ret;
    for (const VariableSet& l : store) {
        ret.push_back(l.front());
    }
    return ret;
}

const Variable* UniqueVarStore::getRep(const Variable* v) {
    for (const VariableSet& l : store) {
        for (const Variable* s : l) {
            if (s == v) {
                return l.front();
            }
        }
    }
    add(v);
    return v;
}

/**
 * Returns the index of the unification-list that contains the given variable.
 * Returns -1, if the variable is not present.
 */
int UniqueVarStore::getIndexOf(const Variable* v) const {
    for (int i = 0; i < static_cast<int>(store.size()); ++i) {
        for (const Variable* c : store[i]) {
            if (c == v) {
                return i;
            }
        }
    }
    return -1;
}

/**
 * ONLY FOR TESTING!
 */
shared_ptr<UniqueVarStore> Query::getUniqueVariableStore() {
    return this->uniqueVarStore;
}
} /* namespace alica */
