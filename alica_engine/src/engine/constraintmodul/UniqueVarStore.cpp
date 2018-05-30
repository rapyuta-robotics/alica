#include "engine/constraintmodul/UniqueVarStore.h"
#include "engine/constraintmodul/Query.h"

#include "engine/constraintmodul/ISolver.h"

#include <assert.h>

namespace alica
{

UniqueVarStore::UniqueVarStore() {}

void UniqueVarStore::clear()
{
    _store.clear();
    _solverVars.clear();
    _agentSolverVars.clear();
}
void UniqueVarStore::initWith(const VariableGrp& vs)
{
    _store.resize(vs.size());
    int i = 0;
    for (const Variable* v : vs) {
        _store[i].push_back(v);
        assert(_store[i].size() == 1);
        ++i;
        // TODO: make sure there is always one element, so we don't need to push back
    }
}
/**
 * Initializes a list with the given variable and put that list into the internal store.
 */
void UniqueVarStore::add(const Variable* v)
{
    VariableGrp l;
    l.push_back(v);
    _store.push_back(std::move(l));
}

void UniqueVarStore::addChecked(const Variable* v)
{
    for (const auto& variables : _store) {
        for (const Variable* variable : variables) {
            if (variable == v) {
                return;
            }
        }
    }
    add(v);
}

/**
 * Add the variable "toAdd" to the front of the list of variables that contains the variable "representing".
 * If such a list does not exist, a new list will be created.
 */
void UniqueVarStore::addVarTo(const Variable* representing, const Variable* toAdd)
{
    for (auto& variables : _store) {
        for (auto& variable : variables) {
            if (representing == variable) {
                variables.insert(variables.begin(), toAdd);
                return;
            }
        }
    }
    VariableGrp nl{toAdd, representing};
    _store.push_back(std::move(nl));
}

void UniqueVarStore::getAllRep(VariableGrp& o_vars) const
{
    for (const VariableGrp& l : _store) {
        o_vars.push_back(l.front());
    }
}

const Variable* UniqueVarStore::getRep(const Variable* v)
{
    for (const VariableGrp& l : _store) {
        for (const Variable* s : l) {
            if (s == v) {
                return l.front();
            }
        }
    }
    add(v);
    return v;
}
SolverVariable* UniqueVarStore::getSolverVariable(const Variable* v) const
{
    assert(_store.size() == _solverVars.size());
    for (int i = 0; i < static_cast<int>(_store.size()); ++i) {
        for (const Variable* s : _store[i]) {
            if (s == v) {
                return _solverVars[i];
            }
        }
    }
    assert(false);
    return nullptr;
}

SolverVariable* UniqueVarStore::getSolverVariable(const DomainVariable* dv, ISolverBase* solver, SolverContext* ctx) const
{
    auto it = _agentSolverVars.find(dv);
    if (it != _agentSolverVars.end()) {
        return it->second;
    }
    assert(false); // creating these on the fly would break ordering
    return nullptr;
}

void UniqueVarStore::setupSolverVars(ISolverBase* solver, SolverContext* ctx, const std::vector<const DomainVariable*>& domainVars)
{
    _solverVars.resize(_store.size());
    for (int i = 0; i < static_cast<int>(_store.size()); ++i) {
        _solverVars[i] = solver->createVariable(_store[i][0]->getId(), ctx);
    }
    for (const DomainVariable* dv : domainVars) {
        _agentSolverVars[dv] = solver->createVariable(dv->getId(), ctx);
    }
}
/**
 * Returns the index of the unification-list that contains the given variable.
 * Returns -1, if the variable is not present.
 */
int UniqueVarStore::getIndexOf(const Variable* v) const
{
    for (int i = 0; i < static_cast<int>(_store.size()); ++i) {
        for (const Variable* c : _store[i]) {
            if (c == v) {
                return i;
            }
        }
    }
    return -1;
}

std::ostream& operator<<(std::ostream& os, const UniqueVarStore& store)
{
    os << "UniqueVarStore: " << std::endl;
    for (const VariableGrp& vs : store._store) {
        os << "Unifications: ";
        for (const Variable* variable : vs) {
            os << *variable << ", ";
        }
        os << std::endl;
    }
    return os;
}

} // namespace alica