#pragma once

#include "engine/Types.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/model/Variable.h"
#include <iostream>
#include <unordered_map>

namespace alica
{

class SolverVariable;
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
    void addChecked(const Variable* v);
    const Variable* getRep(const Variable* v);
    SolverVariable* getSolverVariable(const Variable* v) const;
    SolverVariable* getSolverVariable(const DomainVariable* dv, ISolverBase* solver, SolverContext* ctx);

    void addVarTo(const Variable* representing, const Variable* toAdd);
    void getAllRep(VariableGrp& o_vars) const;
    int getIndexOf(const Variable* v) const;

    void setupSolverVars(ISolverBase* solver, SolverContext* ctx);

    friend std::ostream& operator<<(std::ostream& os, const UniqueVarStore& store);

  private:
    /**
     *  Each inner list of variables is sorted from variables of the top most plan to variables of the deepest plan.
     *  Therefore, the first element is always the variable in the top most plan, where this variable occurs.
     */
    std::vector<VariableGrp> _store;
    std::vector<SolverVariable*> _solverVars;
    std::unordered_map<const DomainVariable*, SolverVariable*> _agentSolverVars;
};

} // namespace alica