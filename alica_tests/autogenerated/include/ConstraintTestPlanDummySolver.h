#pragma once

#include <alica_solver_interface/SolverContext.h>
#include <engine/blackboard/BBIdent.h>
#include <engine/constraintmodul/ISolver.h>
#include <memory>
#include <vector>

namespace alica
{

namespace reasoner
{

class TestDummyContext : public SolverContext
{
  public:
    SolverVariable* createVariable(int64_t id);
    virtual void clear() override;
    const std::vector<std::unique_ptr<SolverVariable>>& getVariables() const { return _vars; }

  private:
    std::vector<std::unique_ptr<SolverVariable>> _vars;
};

class ConstraintTestPlanDummySolver : public ISolver<ConstraintTestPlanDummySolver, BBIdent>
{
  public:
    ConstraintTestPlanDummySolver(AlicaEngine* ae);
    virtual ~ConstraintTestPlanDummySolver();

    bool existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<BBIdent>& results);

    virtual SolverVariable* createVariable(int64_t id, SolverContext* ctx) override;
    virtual std::unique_ptr<SolverContext> createSolverContext() override;

    static int getExistsSolutionCallCounter();
    static int getGetSolutionCallCounter();

  private:
    static int existsSolutionCallCounter;
    static int getSolutionCallCounter;
};
} /* namespace reasoner */

} /* namespace alica */
