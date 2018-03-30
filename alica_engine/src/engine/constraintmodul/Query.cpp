#include <engine/constraintmodul/ConditionStore.h>
#include <engine/constraintmodul/ISolver.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/constraintmodul/ProblemPart.h>
#include <engine/constraintmodul/Query.h>
#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaClock.h"
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

namespace alica {
Query::Query(AlicaEngine* ae) : ae(ae) {
    uniqueVarStore = make_shared<UniqueVarStore>();
}

void Query::addStaticVariable(Variable* v) {
    queriedStaticVariables.push_back(v);
}

void Query::addDomainVariable(const supplementary::AgentID* robot, string ident) {
    queriedDomainVariables.push_back(this->ae->getTeamManager()->getDomainVariable(robot, ident));
}

void Query::clearDomainVariables() {
    queriedDomainVariables.clear();
}

void Query::clearStaticVariables() {
    queriedStaticVariables.clear();
}

bool Query::existsSolution(int solverType, shared_ptr<RunningPlan> rp) {
    ISolver* solver = this->ae->getSolver(solverType);

    vector<shared_ptr<ProblemDescriptor>> cds = vector<shared_ptr<ProblemDescriptor>>();
    vector<Variable*> relevantVariables;
    int domOffset;
    if (!collectProblemStatement(rp, solver, cds, relevantVariables, domOffset)) {
        return false;
    }
    return solver->existsSolution(relevantVariables, cds);
}

bool Query::collectProblemStatement(shared_ptr<RunningPlan> rp, ISolver* solver,
        vector<shared_ptr<ProblemDescriptor>>& pds, vector<Variable*>& relevantVariables, int& domOffset) {
#ifdef Q_DEBUG
    long time = rp->getAlicaEngine()->getIAlicaClock()->now();
#endif
    // Clear variable stuff and calls
    uniqueVarStore->clear();
    relevantStaticVariables.clear();
    relevantDomainVariables.clear();
    problemParts.clear();

    // insert _queried_ variables of this query into the _relevant_ variables
    if (!queriedStaticVariables.empty()) {
        relevantStaticVariables.insert(
                relevantStaticVariables.end(), queriedStaticVariables.begin(), queriedStaticVariables.end());
    }

    if (!queriedDomainVariables.empty()) {
        relevantDomainVariables.insert(
                relevantDomainVariables.end(), queriedDomainVariables.begin(), queriedDomainVariables.end());
    }
#ifdef Q_DEBUG
    std::cout << "Query: Initial relevantStaticVariables Size: " << relevantStaticVariables.size() << std::endl;
    std::cout << "Query: Initial relevantDomainVariables Size: " << relevantDomainVariables.size() << std::endl;
#endif

    // add static variables into the clean unique variable store
    for (auto& v : relevantStaticVariables) {
        uniqueVarStore->add(v);
    }

#ifdef Q_DEBUG
    cout << "Query: Starting Query with static Vars:" << endl << (*this->uniqueVarStore) << endl;
#endif
    // Goes recursive upwards in the plan tree and does three steps on each level.
    while (rp != nullptr && (relevantStaticVariables.size() > 0 || relevantDomainVariables.size() > 0)) {
#ifdef Q_DEBUG
        cout << "Query: Plantree-LVL of " << rp->getPlan()->getName() << endl << (*this->uniqueVarStore) << endl;
#endif

        // 1. fill the query's static and domain variables, as well as its problem parts
        rp->getConstraintStore()->acceptQuery(shared_from_this(), rp);

#ifdef Q_DEBUG
        std::cout << "Query: Size of problemParts is " << this->problemParts.size() << std::endl;
#endif
        // 2. process bindings for plantype
        if (rp->getPlanType() != nullptr) {
            vector<Variable*> tmpVector = vector<Variable*>();
            for (Parametrisation* p : rp->getPlanType()->getParametrisation()) {
                if (p->getSubPlan() == rp->getPlan() &&
                        find(relevantStaticVariables.begin(), relevantStaticVariables.end(), p->getSubVar()) !=
                                relevantStaticVariables.end()) {
                    tmpVector.push_back(p->getVar());
                    uniqueVarStore->addVarTo(p->getSubVar(), p->getVar());
                }
            }
            /**
             * Overwrite relevantStaticVariables for having only the
             * static variables of the current plan tree level for
             * the next iteration. Each while-iteration goes one level up.
             */
            relevantStaticVariables = tmpVector;
        }

        // 3. process bindings for state
        shared_ptr<RunningPlan> parent;
        if (!rp->getParent().expired()) {
            parent = rp->getParent().lock();
        }
        if (parent && parent->getActiveState() != nullptr) {
            vector<Variable*> tmpVector;
            for (Parametrisation* p : parent->getActiveState()->getParametrisation()) {
                if ((p->getSubPlan() == rp->getPlan() || p->getSubPlan() == rp->getPlanType()) &&
                        find(relevantStaticVariables.begin(), relevantStaticVariables.end(), p->getSubVar()) !=
                                relevantStaticVariables.end()) {
                    tmpVector.push_back(p->getVar());
                    uniqueVarStore->addVarTo(p->getSubVar(), p->getVar());
                }
            }
            /**
             * Overwrite relevantStaticVariables for having only the
             * static variables of the current plan tree level for
             * the next iteration. Each while-iteration goes one level up.
             */
            relevantStaticVariables = tmpVector;
        }
        rp = parent;
    }
#ifdef Q_DEBUG
    cout << "Query: " << (*this->uniqueVarStore) << endl;
#endif
    // now we have a vector<ProblemPart> in problemParts ready to be queried together with a store of unifications
    if (problemParts.size() == 0) {
#ifdef Q_DEBUG
        cout << "Query: Empty Query!" << endl;
#endif
        return false;
    }

#ifdef Q_DEBUG
    std::cout << "Query: Size of problemParts is " << this->problemParts.size() << std::endl;
#endif

    for (shared_ptr<ProblemPart> probPart : problemParts) {
        vector<Variable*> staticCondVariables = vector<Variable*>(probPart->getCondition()->getVariables());

        // create a vector of solver variables from the static condition variables
        auto staticSolverVars = make_shared<vector<shared_ptr<SolverVariable>>>();
        staticSolverVars->reserve(staticCondVariables.size());
        for (auto& variable : staticCondVariables) {
            auto representingVariable = uniqueVarStore->getRep(variable);
            if (representingVariable->getSolverVar() == nullptr) {
                representingVariable->setSolverVar(solver->createVariable(representingVariable->getId()));
            }

            staticSolverVars->push_back(representingVariable->getSolverVar());
        }

        // create a vector of solver variables from the domain variables of the currently iterated problem part
        auto domainSolverVars =
                make_shared<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>>>>();
        auto agentsInScope = make_shared<vector<shared_ptr<vector<const supplementary::AgentID*>>>>();
        for (int j = 0; j < probPart->getDomainVariables()->size(); ++j) {
            auto ll = make_shared<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>>();
            agentsInScope->push_back(probPart->getAgentsInScope()->at(j));
            domainSolverVars->push_back(ll);
            for (auto domainVars : probPart->getDomainVariables()->at(j)) {
                auto domainSolverVars = make_shared<vector<shared_ptr<SolverVariable>>>();
                domainSolverVars->reserve(domainVars.size());
                for (int i = 0; i < domainVars.size(); ++i) {
                    if (domainVars.at(i)->getSolverVar() == nullptr) {
                        domainVars.at(i)->setSolverVar(solver->createVariable(domainVars.at(i)->getId()));
                    }
                    domainSolverVars->push_back(domainVars.at(i)->getSolverVar());
                }
                ll->push_back(domainSolverVars);
            }
        }

        // fill a new problem descriptor and put it into the out-parameter "pds"
        auto pd = make_shared<ProblemDescriptor>(staticSolverVars, domainSolverVars);
        pd->setAgentsInScope(agentsInScope);
        probPart->getCondition()->getConstraint(
                pd, probPart->getRunningPlan());  // this insert the actual problem description
        pds.push_back(pd);
    }

    // write all static variables into the out-parameter "relevantVariables"
    relevantVariables = uniqueVarStore->getAllRep();

    // the index of the first domain variable after the static variables
    domOffset = relevantVariables.size();

    // write all domain variables into the out-parameter "relevantVariables"
    if (!relevantDomainVariables.empty()) {
        relevantVariables.insert(
                relevantVariables.end(), relevantDomainVariables.begin(), relevantDomainVariables.end());
    }

#ifdef Q_DEBUG
    std::cout << "Query: Number of relevant static variables: " << domOffset << std::endl;
    std::cout << "Query: Number of relevant domain variables: " << relevantDomainVariables.size() << std::endl;
    std::cout << "Query: Total number of relevant variables: " << relevantVariables.size() << std::endl;
    cout << "Query: PrepTime: " << (ae->getIAlicaClock()->now() - time) / 10000.0 << endl;
#endif
    return true;
}

vector<Variable*> Query::getRelevantStaticVariables() {
    return relevantStaticVariables;
}

void Query::setRelevantStaticVariables(vector<Variable*> value) {
    relevantStaticVariables = value;
}

vector<Variable*> Query::getRelevantDomainVariables() {
    return relevantDomainVariables;
}

void Query::setRelevantDomainVariables(vector<Variable*> value) {
    relevantDomainVariables = value;
}

void Query::addProblemParts(vector<shared_ptr<ProblemPart>>& l) {
    problemParts.insert(problemParts.end(), l.begin(), l.end());
}

/// Part for the implementation of the internal class UniqueVarStore

UniqueVarStore::UniqueVarStore() {
    store = vector<vector<Variable*>>();
}

void UniqueVarStore::clear() {
    store.clear();
}

/**
 * Initializes a list with the given variable and put that list into the internal store.
 */
void UniqueVarStore::add(Variable* v) {
    vector<Variable*> l = vector<Variable*>();
    l.push_back(v);
    store.push_back(l);
}

/**
 * Add the variable "toAdd" to the front of the list of variables that contains the variable "representing".
 * If such a list does not exist, a new list will be created.
 */
void UniqueVarStore::addVarTo(Variable* representing, Variable* toAdd) {
    for (auto& variables : store) {
        for (auto& variable : variables) {
            if (representing == variable) {
                variables.insert(variables.begin(), toAdd);
                return;
            }
        }
    }
    vector<Variable*> nl;
    nl.insert(nl.begin(), representing);
    nl.insert(nl.begin(), toAdd);
    store.push_back(nl);
}

vector<Variable*> UniqueVarStore::getAllRep() {
    vector<Variable*> ret = vector<Variable*>();
    for (vector<Variable*> l : store) {
        ret.push_back(l.front());
    }
    return ret;
}

Variable* UniqueVarStore::getRep(Variable* v) {
    for (vector<Variable*> l : store) {
        for (Variable* s : l) {
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
int UniqueVarStore::getIndexOf(Variable* v) {
    for (int i = 0; i < store.size(); ++i) {
        for (Variable* c : store[i]) {
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
