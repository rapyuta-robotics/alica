#include "engine/model/Condition.h"
#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"
#include <engine/constraintmodul/ConditionStore.h>
#include <engine/constraintmodul/ProblemPart.h>
#include <engine/constraintmodul/Query.h>

#include <iostream>

//#define CS_DEBUG

namespace alica {

/**
 * Default constructor
 */
ConditionStore::ConditionStore() {}

ConditionStore::~ConditionStore() {}

/**
 * Clear store, revoking all constraints
 */
void ConditionStore::clear() {
    activeVar2CondMap.clear();
    mtx.lock();
    activeConditions.clear();
    mtx.unlock();
}

/**
 * Add a condition to the store
 * @param con A Condition
 */
void ConditionStore::addCondition(const Condition* con) {
    if (con == nullptr || (con->getVariables().size() == 0 && con->getQuantifiers().size() == 0)) {
        return;
    }

    bool modified = false;
    mtx.lock();
    if (find(activeConditions.begin(), activeConditions.end(), con) == activeConditions.end()) {
        modified = true;
        activeConditions.push_back(con);
    }
    mtx.unlock();
    if (modified) {
        for (const Variable* variable : con->getVariables()) {
            auto it = activeVar2CondMap.find(variable);
            if (it != activeVar2CondMap.end()) {
                it->second->push_back(con);
            } else {
                auto condList = make_shared<vector<const Condition*>>();
                condList->push_back(con);
                activeVar2CondMap.emplace(variable, condList);
            }
        }
    }
#ifdef CS_DEBUG
    cout << "CS: Added condition in " << con->getAbstractPlan()->getName() << " with " << con->getVariables().size()
         << " variables. CS: " << this << endl;
#endif
}

/**
 * Revoke a specific condition from the constraint store
 *
 * @param con The condition to be removed
 */
void ConditionStore::removeCondition(const Condition* con) {
    if (con == nullptr) {
        return;
    }
    bool modified = false;
    mtx.lock();
    if (find(activeConditions.begin(), activeConditions.end(), con) != activeConditions.end()) {
        modified = true;
        activeConditions.remove(con);
    }
    mtx.unlock();
    if (modified) {
        for (const Variable* v : con->getVariables()) {
            activeVar2CondMap[v]->erase(std::remove(activeVar2CondMap[v]->begin(), activeVar2CondMap[v]->end(), con),
                    activeVar2CondMap[v]->end());
        }
    }

#ifdef CS_DEBUG
    std::cout << "CS: Removed condition in " << con->getAbstractPlan()->getName() << " with "
              << con->getVariables().size() << " variables." << std::endl;
#endif
}

/**
 * Writes static and domain variables, as well as, problem parts into the query.
 */
void ConditionStore::acceptQuery(Query& query, shared_ptr<RunningPlan> rp) const {
#ifdef CS_DEBUG
    std::cout << "ConditionStore: Accepting Query - Active conditions in store is " << activeConditions.size() <<" CS: "<< this
              << std::endl;
#endif
    if (activeConditions.empty()) {
        return;
    }

    std::vector<const Variable*> staticVarsToCheck = query.getRelevantStaticVariables();
    std::vector<const Variable*> domVarsToCheck = query.getRelevantDomainVariables();
    if (staticVarsToCheck.size() == 0 && domVarsToCheck.size() == 0) {
        return;  // nothing to do
    }

#ifdef CS_DEBUG
    std::cout << "ConditionStore: Query contains static variables: ";
    for (const Variable* v : staticVarsToCheck) {
        std::cout << v->getName() << "(" << v->getId() << "), ";
    }
    std::cout << std::endl;
#endif

    std::map<const Condition*, shared_ptr<ProblemPart>> newCondProbPartMap;
    std::map<const Condition*, shared_ptr<ProblemPart>> allCondProbPartMap;
    {
        std::lock_guard<std::mutex> lock(mtx);
        for (const Condition* cond : activeConditions) {
            allCondProbPartMap.emplace(cond, make_shared<ProblemPart>(cond, rp));
        }
    }

    std::vector<const Variable*> staticVarsChecked;
    std::vector<const Variable*> domVarsChecked;
    while (newCondProbPartMap.size() < allCondProbPartMap.size() &&
            (domVarsToCheck.size() > 0 || staticVarsToCheck.size() > 0)) {
        if (staticVarsToCheck.size() > 0) {
            const Variable* curStaticVariable = staticVarsToCheck[staticVarsToCheck.size() - 1];
            staticVarsToCheck.pop_back();
            staticVarsChecked.push_back(curStaticVariable);

#ifdef CS_DEBUG
            std::cout << "ConditionStore: Checking static variable: " << *curStaticVariable << std::endl;
#endif

            auto activeVar2CondMapEntry = activeVar2CondMap.find(curStaticVariable);
            if (activeVar2CondMapEntry == activeVar2CondMap.end()) {
                // the current variable wasn't active
                continue;
            }

#ifdef CS_DEBUG
            std::cout << "ConditionStore: Conditions active under variable " << (*activeVar2CondMapEntry->first) << ": "
                      << activeVar2CondMapEntry->second->size() << std::endl;
#endif
            for (const Condition* c : *(activeVar2CondMapEntry->second)) {
                if (newCondProbPartMap.find(c) != newCondProbPartMap.end()) {
                    // condition was already inserted into the newCondProbPartMap
                    continue;
                }

                shared_ptr<ProblemPart> problemPart = allCondProbPartMap[c];
                newCondProbPartMap.emplace(c, problemPart);
                /**
                 *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
                 * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
                 * 2. List of Robots, e.g., An agent has variables X,Y.
                 * 3. Vector of Variables, e.g., variables X,Y.
                 * 4. Variable, e.g., variable X.
                 */
                auto domainVariables = problemPart->getDomainVariables();
                for (auto& listOfRobots : (*domainVariables)) {
                    for (const std::vector<const Variable*>& variables : listOfRobots) {
                        for (auto variable : variables) {
                            if (find(domVarsChecked.begin(), domVarsChecked.end(), variable) == domVarsChecked.end() &&
                                    find(domVarsToCheck.begin(), domVarsToCheck.end(), variable) ==
                                            domVarsToCheck.end()) {
                                std::cout << "CS: Adding DomVar : " << *variable << std::endl;
                                domVarsToCheck.push_back(variable);
                            }
                        }
                    }
                }
                for (const Variable* variable : c->getVariables()) {
                    if (find(staticVarsChecked.begin(), staticVarsChecked.end(), variable) == staticVarsChecked.end() &&
                            find(staticVarsToCheck.begin(), staticVarsToCheck.end(), variable) ==
                                    staticVarsToCheck.end()) {
                        staticVarsToCheck.push_back(variable);
                    }
                }
            }
        } else if (domVarsToCheck.size() > 0) {
            const Variable* curDomainVariable = domVarsToCheck[domVarsToCheck.size() - 1];
            domVarsToCheck.pop_back();
            domVarsChecked.push_back(curDomainVariable);

            for (auto& condProbPartPair : allCondProbPartMap) {
                if (newCondProbPartMap.find(condProbPartPair.first) != newCondProbPartMap.end()) {
                    // condition was already in the newCondProbPartMap
                    continue;
                }

                if (!condProbPartPair.second->hasVariable(curDomainVariable)) {
                    // curDomainVariable does not exist in the problem part
                    continue;
                }

                newCondProbPartMap.emplace(condProbPartPair.first, condProbPartPair.second);

                /**
                 *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
                 * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
                 * 2. List of Robots, e.g., An agent has variables X,Y.
                 * 3. Vector of Variables, e.g., variables X,Y.
                 * 4. Variable, e.g., variable X.
                 */
                for (auto& listOfRobots : (*condProbPartPair.second->getDomainVariables())) {
                    for (auto& variables : listOfRobots) {
                        for (auto& variable : variables) {
                            if (find(domVarsChecked.begin(), domVarsChecked.end(), variable) == domVarsChecked.end() &&
                                    find(domVarsToCheck.begin(), domVarsToCheck.end(), variable) ==
                                            domVarsToCheck.end()) {
                                domVarsToCheck.push_back(variable);
                            }
                        }
                    }
                }

                for (const Variable* variable : condProbPartPair.first->getVariables()) {
                    if (find(staticVarsChecked.begin(), staticVarsChecked.end(), variable) == staticVarsChecked.end() &&
                            find(staticVarsToCheck.begin(), staticVarsToCheck.end(), variable) ==
                                    staticVarsToCheck.end()) {
                        staticVarsToCheck.push_back(variable);
                    }
                }
            }
        }
    }
    if (!staticVarsChecked.empty()) {
        staticVarsChecked.insert(staticVarsChecked.end(), staticVarsToCheck.begin(), staticVarsToCheck.end());
    }

    if (!domVarsToCheck.empty()) {
        domVarsChecked.insert(domVarsChecked.end(), domVarsToCheck.begin(), domVarsToCheck.end());
    }

    // write back relevant variables, this contains variables obtained earlier
    query.setRelevantStaticVariables(staticVarsChecked);
    query.setRelevantDomainVariables(domVarsChecked);

    // write back problem parts
    vector<shared_ptr<ProblemPart>> problemParts;
    for (auto& pair : newCondProbPartMap) {
        auto problemPart = pair.second;
        if (find(problemParts.begin(), problemParts.end(), problemPart) == problemParts.end()) {
            problemParts.push_back(problemPart);
        }
    }
    query.addProblemParts(problemParts);
}

}  // namespace alica
