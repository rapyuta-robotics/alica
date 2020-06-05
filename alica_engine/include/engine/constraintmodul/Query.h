#pragma once

#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanInterface.h"
#include "engine/TeamObserver.h"
#include "engine/constraintmodul/ConditionStore.h"
#include "engine/constraintmodul/ISolver.h"
#include "engine/constraintmodul/ProblemDescriptor.h"
#include "engine/constraintmodul/ProblemPart.h"
#include "engine/constraintmodul/UniqueVarStore.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/model/Condition.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/VariableBinding.h"
#include "engine/model/PlanType.h"
#include "engine/model/State.h"

#include <alica_solver_interface/SolverContext.h>

#include <map>
#include <memory>
#include <vector>

namespace alica
{
class ProblemPart;
class RunningPlan;
class SolverContext;

template <class T>
class BufferedSet
{
public:
    BufferedSet() = default;
    BufferedSet(const BufferedSet&) = delete;
    BufferedSet& operator=(const BufferedSet&) = delete;

    const std::vector<T>& getCurrent() const { return _current; }
    std::vector<T>& editCurrent() { return _current; }
    std::vector<T>& editNext() { return _next; }
    void flip() { std::swap(_current, _next); }
    void mergeAndFlip()
    {
        if (!_current.empty()) {
            _next.insert(_next.end(), _current.begin(), _current.end());
        }
        _current.clear();
        flip();
    }
    void clear()
    {
        _current.clear();
        _next.clear();
    }
    bool hasCurrentlyAny() const { return !_current.empty(); }
    bool hasCurrently(T v) const { return std::find(_current.begin(), _current.end(), v) != _current.end(); }
    bool has(T v) const { return hasCurrently(v) || (std::find(_next.begin(), _next.end(), v) != _next.end()); }

private:
    std::vector<T> _current;
    std::vector<T> _next;
};

using BufferedVariableGrp = BufferedSet<const Variable*>;
using BufferedDomainVariableGrp = BufferedSet<const DomainVariable*>;

/**
 * Encapsulates queries to variables (which are associated with specific solvers).
 */
class Query
{
public:
    Query();

    void addStaticVariable(const alica::Variable* v);
    void addDomainVariable(essentials::IdentifierConstPtr robot, const std::string& ident, const AlicaEngine* ae);
    void clearDomainVariables();
    void clearStaticVariables();

    template <class SolverType>
    bool existsSolution(ThreadSafePlanInterface pi);

    template <class SolverType, typename ResultType>
    bool getSolution(ThreadSafePlanInterface pi, std::vector<ResultType>& result);

    BufferedVariableGrp& editStaticVariableBuffer() { return _staticVars; }
    BufferedDomainVariableGrp& editDomainVariableBuffer() { return _domainVars; }

    void addProblemPart(ProblemPart&& p);
    int getPartCount() const { return _problemParts.size(); }
    const std::vector<ProblemPart>& getProblemParts() const { return _problemParts; }
    const UniqueVarStore& getUniqueVariableStore() const; /*< for testing only!!! */

private:
    void clearTemporaries();
    void fillBufferFromQuery();
    bool collectProblemStatement(ThreadSafePlanInterface pi, ISolverBase& solver, std::vector<std::shared_ptr<ProblemDescriptor>>& cds, int& domOffset);

    VariableGrp _queriedStaticVariables;
    DomainVariableGrp _queriedDomainVariables;

    UniqueVarStore _uniqueVarStore;
    std::vector<ProblemPart> _problemParts;

    BufferedVariableGrp _staticVars;
    BufferedDomainVariableGrp _domainVars;

    VariableGrp _relevantVariables;
    std::unique_ptr<SolverContext> _context;
};

template <class SolverType>
bool Query::existsSolution(ThreadSafePlanInterface pi)
{
    SolverType& solver = pi.getAlicaEngine()->getSolver<SolverType>();

    std::vector<std::shared_ptr<ProblemDescriptor>> cds;
    int domOffset;
    if (!collectProblemStatement(pi, solver, cds, domOffset)) {
        return false;
    }
    return solver->existsSolution(_context.get(), cds);
}

template <class SolverType, typename ResultType>
bool Query::getSolution(ThreadSafePlanInterface pi, std::vector<ResultType>& result)
{
    result.clear();

    // Collect the complete problem specification
    std::vector<std::shared_ptr<ProblemDescriptor>> cds;
    int domOffset;

    if (!pi.getAlicaEngine()->existSolver<SolverType>()) {
        std::cerr << "Query::getSolution: The engine does not have a suitable solver for the given type available." << std::endl;
        return false;
    }

    SolverType& solver = pi.getAlicaEngine()->getSolver<SolverType>();
    if (!collectProblemStatement(pi, solver, cds, domOffset)) {
        return false;
    }

    std::cout << "Query: " << _uniqueVarStore << std::endl;

    // TODO: get rid of the interrim vector (see below how)
    std::vector<ResultType> solverResult;
    // let the solver solve the problem
    bool ret = solver.getSolution(_context.get(), cds, solverResult);

    if (ret && solverResult.size() > 0) {
        int i = 0;
        VariableSyncModule& rs = pi.getAlicaEngine()->editResultStore();
        for (const ResultType& value : solverResult) {
            rs.postResult(_relevantVariables[i]->getId(), Variant(value));
            ++i;
        }

        // TODO this can be done in place. The queried static should be at the beginning of the array anyway
        // create a result vector that is filtered by the queried variables
        for (const Variable* staticVariable : _queriedStaticVariables) {
            int idx = _uniqueVarStore.getIndexOf(staticVariable);
            assert(idx >= 0);
            result.push_back(solverResult[idx]);
        }
        // Again, the queried domain variables should be at the beginning of the domain variable segment
        // So a simple move and resize should do the trick
        for (int i = 0; i < static_cast<int>(_queriedDomainVariables.size()); ++i) {
            for (int j = domOffset; j < static_cast<int>(_relevantVariables.size()); ++j) {
                if (_relevantVariables[j] == _queriedDomainVariables[i]) {
                    result.push_back(solverResult[j]);
                    break;
                }
            }
        }
    }
    return ret;
}
} /* namespace alica */
