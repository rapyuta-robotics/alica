#pragma once

#include <engine/blackboard/BBIdent.h>
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

class DummySolver : public alica::ISolver<DummySolver, BBIdent>
{
  public:
    DummySolver(AlicaEngine* ae);
    virtual ~DummySolver();

    bool existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<BBIdent>& results);
    virtual SolverVariable* createVariable(int64_t representingVariableId, SolverContext* ctx) override;
    virtual std::unique_ptr<SolverContext> createSolverContext() override;

  private:
    const std::string& getValue(int64_t id, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls) const;
};
} /* namespace reasoner */
} /* namespace alica */
