#pragma once

#include <engine/constraintmodul/ISolver.h>
#include <memory>
#include <vector>

namespace alica
{

class AlicaEngine;
class ProblemDescriptor;
class Variable;
class SolverVariable;
class SolverContext;

namespace reasoner
{

class SimpleSolver : public alica::ISolver<SimpleSolver, int64_t>
{
  public:
    SimpleSolver(AlicaEngine* ae);
    virtual ~SimpleSolver();

    bool existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<int64_t>& results);
    virtual SolverVariable* createVariable(int64_t representingVariableId, SolverContext* ctx) override;
    virtual std::unique_ptr<SolverContext> createSolverContext() override;

  private:
    const std::string& getValue(int64_t id, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls) const;
};
} /* namespace reasoner */
} /* namespace alica */
