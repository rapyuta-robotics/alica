#include "engine/constraintmodul/UniqueVarStore.h"
#include "engine/constraintmodul/Query.h"
#include <assert.h>

namespace alica
{

UniqueVarStore::UniqueVarStore() {}

void UniqueVarStore::clear()
{
    _store.clear();
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
    VariableGrp nl;
    nl.insert(nl.begin(), representing);
    nl.insert(nl.begin(), toAdd);
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

/**
 * ONLY FOR TESTING!
 */
const UniqueVarStore& Query::getUniqueVariableStore() const
{
    return _uniqueVarStore;
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