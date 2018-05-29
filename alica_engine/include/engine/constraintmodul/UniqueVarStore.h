#pragma once

#include "engine/Types.h"
#include "engine/model/Variable.h"
#include <iostream>

namespace alica
{
/**
 * Internal class to deal with bindings in states and plantypes
 */
class UniqueVarStore
{
  public:
    UniqueVarStore();

    void clear();
    void initWith(const VariableGrp& vs);
    void add(const Variable* v);
    const Variable* getRep(const Variable* v);
    void addVarTo(const Variable* representing, const Variable* toAdd);
    void getAllRep(VariableGrp& o_vars) const;
    int getIndexOf(const Variable* v) const;
    friend std::ostream& operator<<(std::ostream& os, const UniqueVarStore& store);

  private:
    /**
     *  Each inner list of variables is sorted from variables of the top most plan to variables of the deepest plan.
     *  Therefore, the first element is always the variable in the top most plan, where this variable occurs.
     */
    std::vector<VariableGrp> _store;
};

} // namespace alica