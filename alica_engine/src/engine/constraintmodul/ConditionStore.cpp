#include "engine/model/Condition.h"
#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/Quantifier.h"
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
    _activeVar2CondMap.clear();
    _mtx.lock();
    _activeConditions.clear();
    _mtx.unlock();
}

/**
 * Add a condition to the store
 * @param con A Condition
 */
void ConditionStore::addCondition(const Condition* con) {
    if (con == nullptr || (con->getVariables().empty() && con->getQuantifiers().empty())) {
        return;
    }

    bool modified = false;
    _mtx.lock();
    if (std::find(_activeConditions.begin(), _activeConditions.end(), con) == _activeConditions.end()) {
        modified = true;
        _activeConditions.push_back(con);
    }
    _mtx.unlock();
    if (modified) {
        for (const Variable* variable : con->getVariables()) {
            _activeVar2CondMap[variable].push_back(con);
        }
        for (const Quantifier* qv : con->getQuantifiers()) {
            for (const Variable* variable : qv->getTemplateVariables()) {
                _activeVar2CondMap[variable].push_back(con);
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
    _mtx.lock();
    ConditionSet::iterator cit = find(_activeConditions.begin(), _activeConditions.end(), con);
    if (cit != _activeConditions.end()) {
        modified = true;
        _activeConditions.erase(cit);
    }
    _mtx.unlock();
    if (modified) {
        for (const Variable* v : con->getVariables()) {
            auto it = _activeVar2CondMap.find(v);
            it->second.erase(std::remove(it->second.begin(), it->second.end(), con), it->second.end());
        }
        for (const Quantifier* qv : con->getQuantifiers()) {
            for (const Variable* v : qv->getTemplateVariables()) {
                auto it = _activeVar2CondMap.find(v);
                it->second.erase(std::remove(it->second.begin(), it->second.end(), con), it->second.end());
            }
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
    std::cout << "ConditionStore: Accepting Query - Active conditions in store is " << _activeConditions.size()
              << " CS: " << this << std::endl;
#endif
    if (_activeConditions.empty()) {
        return;
    }

    BufferedVariableGrp& staticVarBuffer = query.editStaticVariableBuffer();
    BufferedDomainVariableGrp& domainVarBuffer = query.editDomainVariableBuffer();
    if (!staticVarBuffer.hasCurrentlyAny() && !domainVarBuffer.hasCurrentlyAny()) {
        return;  // nothing to do
    }

#ifdef CS_DEBUG
    std::cout << "ConditionStore: Query contains static variables: ";
    for (const Variable* v : staticVarBuffer.getCurrent()) {
        std::cout << v->getName() << "(" << v->getId() << "), ";
    }
    std::cout << std::endl;
#endif
    const int previousPartCount = query.getPartCount();

    while (query.getPartCount() - previousPartCount < static_cast<int>(_activeConditions.size()) &&
            (domainVarBuffer.hasCurrentlyAny() || staticVarBuffer.hasCurrentlyAny())) {
        if (staticVarBuffer.hasCurrentlyAny()) {
            const Variable* curStaticVariable = staticVarBuffer.getCurrent().back();
            staticVarBuffer.editCurrent().pop_back();
            staticVarBuffer.editNext().push_back(curStaticVariable);

#ifdef CS_DEBUG
            std::cout << "ConditionStore: Checking static variable: " << *curStaticVariable << std::endl;
#endif

            auto activeVar2CondMapEntry = _activeVar2CondMap.find(curStaticVariable);
            if (activeVar2CondMapEntry == _activeVar2CondMap.end()) {
                // the current variable wasn't active
                continue;
            }

#ifdef CS_DEBUG
            std::cout << "ConditionStore: Conditions active under variable " << (*activeVar2CondMapEntry->first) << ": "
                      << activeVar2CondMapEntry->second->size() << std::endl;
#endif
            for (const Condition* c : activeVar2CondMapEntry->second) {
                if (std::find_if(query.getProblemParts().begin() + previousPartCount, query.getProblemParts().end(),
                            [c](const ProblemPart& pp) { return pp.getCondition() == c; }) !=
                        query.getProblemParts().end()) {
                    // condition was already inserted
                    continue;
                }
                query.addProblemPart(ProblemPart(c, rp));
            }
        } else if (domainVarBuffer.hasCurrentlyAny()) {
            const DomainVariable* curDomainVariable = domainVarBuffer.getCurrent().back();
            domainVarBuffer.editCurrent().pop_back();
            domainVarBuffer.editNext().push_back(curDomainVariable);

            auto activeVar2CondMapEntry = _activeVar2CondMap.find(curDomainVariable->getTemplateVariable());
            if (activeVar2CondMapEntry == _activeVar2CondMap.end()) {
                // the current variable wasn't active
                continue;
            }
            for (const Condition* c : activeVar2CondMapEntry->second) {
                if (std::find_if(query.getProblemParts().begin() + previousPartCount, query.getProblemParts().end(),
                            [c](const ProblemPart& pp) { return pp.getCondition() == c; }) !=
                        query.getProblemParts().end()) {
                    // condition was already inserted
                    continue;
                }
                // if c has a quantifier that currently covers the agent & has the right tempalte var, add it
                for (const Quantifier* q : c->getQuantifiers()) {
                    if (q->hasTemplateVariable(curDomainVariable->getTemplateVariable()) &&
                            q->isAgentInScope(curDomainVariable->getAgent(), rp)) {
                        query.addProblemPart(ProblemPart(c, rp));
                        break;
                    }
                }
            }
        }
    }

    // write back relevant variables, this contains variables obtained earlier
    staticVarBuffer.mergeAndFlip();
    domainVarBuffer.mergeAndFlip();
}

}  // namespace alica
