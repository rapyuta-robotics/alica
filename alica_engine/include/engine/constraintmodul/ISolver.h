#pragma once

#include "engine/Types.h"
#include <memory>

namespace alica
{
class AlicaEngine;
class ProblemDescriptor;
class Variable;
class SolverVariable;
class SolverContext;

class ISolverBase
{
  public:
    ISolverBase(AlicaEngine* ae)
        : _ae(ae)
    {
    }
    virtual ~ISolverBase() {}
    virtual SolverVariable* createVariable(int64_t id, SolverContext* ctx) = 0;
    virtual std::unique_ptr<SolverContext> createSolverContext() = 0;

    virtual double utilityEstimate(const VariableGrp& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls) { return 0; }

  protected:
    AlicaEngine* getAlicaEngine() const { return _ae; }

  private:
    AlicaEngine* _ae;
};

template <class SolverType, typename ResultType>
class ISolver : public ISolverBase
{
  public:
    ISolver(AlicaEngine* ae)
        : ISolverBase(ae)
    {
    }
    virtual ~ISolver() {}

    bool existsSolution(const VariableGrp& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls)
    {
        return static_cast<SolverType*>(this)->existsSolutionImpl(vars, calls);
    }

    bool getSolution(const VariableGrp& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<ResultType>& results)
    {
        return static_cast<SolverType*>(this)->getSolutionImpl(vars, calls, results);
    }
};

} /* namespace alica */
