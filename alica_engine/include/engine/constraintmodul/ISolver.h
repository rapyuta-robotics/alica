#pragma once

#include "engine/Types.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include <memory>

namespace alica
{
class ProblemDescriptor;
class Variable;
class SolverVariable;
class SolverContext;

class ISolverBase
{
public:
    ISolverBase(VariableSyncModule* vsm)
            : _vsm(vsm)
    {
    }
    virtual ~ISolverBase() {}
    virtual SolverVariable* createVariable(int64_t id, SolverContext* ctx) = 0;
    virtual std::unique_ptr<SolverContext> createSolverContext() = 0;

protected:
    VariableSyncModule* getResultStore() { return _vsm; }

private:
    VariableSyncModule* _vsm;
};

template <class SolverType, typename ResultType>
class ISolver : public ISolverBase
{
public:
    ISolver(VariableSyncModule* vsm)
            : ISolverBase(vsm)
    {
    }
    virtual ~ISolver() {}

    bool existsSolution(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls)
    {
        return static_cast<SolverType*>(this)->existsSolutionImpl(ctx, calls);
    }

    bool getSolution(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<ResultType>& results)
    {
        return static_cast<SolverType*>(this)->getSolutionImpl(ctx, calls, results);
    }
};

} /* namespace alica */
