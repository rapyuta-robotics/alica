#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Condition.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/Quantifier.h"
#include "engine/model/Variable.h"
#include <engine/constraintmodul/ConditionStore.h>
#include <engine/constraintmodul/ProblemPart.h>
#include <engine/constraintmodul/Query.h>

#include <alica_common_config/debug_output.h>

#include <iostream>

namespace alica
{
namespace
{
std::ostream& operator<<(std::ostream& out, const VariableGrp& vg)
{
    for (const Variable* v : vg) {
        out << v->getName() << "(" << v->getId() << "), ";
    }
    return out;
}
}
/**
 * Default constructor
 */
ConditionStore::ConditionStore() {}

ConditionStore::~ConditionStore() {}

/**
 * Clear store, revoking all constraints
 */
void ConditionStore::clear()
{
    std::lock_guard<std::mutex> lock(_mtx);
    _activeVar2CondMap.clear();
    _activeConditions.clear();
}

/**
 * Add a condition to the store
 * @param con A Condition
 */
void ConditionStore::addCondition(const Condition* con)
{
    if (con == nullptr || (con->getVariables().empty() && con->getQuantifiers().empty())) {
        return;
    }

    bool modified = false;
    std::lock_guard<std::mutex> lock(_mtx);
    if (std::find(_activeConditions.begin(), _activeConditions.end(), con) == _activeConditions.end()) {
        modified = true;
        _activeConditions.push_back(con);
    }
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

    ALICA_DEBUG_MSG("CS: Added condition in " << con->getAbstractPlan()->getName() << " with " << con->getVariables().size() << " variables. CS: " << this);
}

/**
 * Revoke a specific condition from the constraint store
 *
 * @param con The condition to be removed
 */
void ConditionStore::removeCondition(const Condition* con)
{
    if (con == nullptr) {
        assert(false);
        return;
    }
    bool modified = false;
    std::lock_guard<std::mutex> lock(_mtx);
    ConditionGrp::iterator cit = find(_activeConditions.begin(), _activeConditions.end(), con);
    if (cit != _activeConditions.end()) {
        modified = true;
        _activeConditions.erase(cit);
    }
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

    ALICA_DEBUG_MSG("CS: Removed condition in " << con->getAbstractPlan()->getName() << " with " << con->getVariables().size() << " variables.");
}

/**
 * Writes static and domain variables, as well as, problem parts into the query.
 */
void ConditionStore::acceptQuery(Query& query, const RunningPlan* rp) const
{
    ALICA_DEBUG_MSG("ConditionStore: Accepting Query - Active conditions in store is " << _activeConditions.size() << " CS: " << this);
    if (_activeConditions.empty()) {
        return;
    }

    BufferedVariableGrp& staticVarBuffer = query.editStaticVariableBuffer();
    BufferedDomainVariableGrp& domainVarBuffer = query.editDomainVariableBuffer();
    if (!staticVarBuffer.hasCurrentlyAny() && !domainVarBuffer.hasCurrentlyAny()) {
        return; // nothing to do
    }

    ALICA_DEBUG_MSG("ConditionStore: Query contains static variables: " << staticVarBuffer.getCurrent());

    const int previousPartCount = query.getPartCount();

    std::lock_guard<std::mutex> lock(_mtx);
    while (query.getPartCount() - previousPartCount < static_cast<int>(_activeConditions.size()) &&
            (domainVarBuffer.hasCurrentlyAny() || staticVarBuffer.hasCurrentlyAny())) {
        if (staticVarBuffer.hasCurrentlyAny()) {
            const Variable* curStaticVariable = staticVarBuffer.getCurrent().back();
            staticVarBuffer.editCurrent().pop_back();
            staticVarBuffer.editNext().push_back(curStaticVariable);

            ALICA_DEBUG_MSG("ConditionStore: Checking static variable: " << *curStaticVariable);

            auto activeVar2CondMapEntry = _activeVar2CondMap.find(curStaticVariable);
            if (activeVar2CondMapEntry == _activeVar2CondMap.end()) {
                // the current variable wasn't active
                continue;
            }

            ALICA_DEBUG_MSG(
                    "ConditionStore: Conditions active under variable " << *activeVar2CondMapEntry->first << ": " << activeVar2CondMapEntry->second.size());

            for (const Condition* c : activeVar2CondMapEntry->second) {
                if (std::find_if(query.getProblemParts().begin() + previousPartCount, query.getProblemParts().end(),
                            [c](const ProblemPart& pp) { return pp.getCondition() == c; }) != query.getProblemParts().end()) {
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
                            [c](const ProblemPart& pp) { return pp.getCondition() == c; }) != query.getProblemParts().end()) {
                    // condition was already inserted
                    continue;
                }
                // if c has a quantifier that currently covers the agent & has the right tempalte var, add it
                for (const Quantifier* q : c->getQuantifiers()) {
                    if (q->hasTemplateVariable(curDomainVariable->getTemplateVariable()) && q->isAgentInScope(curDomainVariable->getAgent(), *rp)) {
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

} // namespace alica
