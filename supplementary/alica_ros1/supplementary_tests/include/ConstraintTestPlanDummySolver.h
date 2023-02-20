#pragma once

#include <engine/constraintmodul/ISolver.h>
#include <engine/constraintmodul/VariableSyncModule.h>
#include <memory>
#include <vector>

namespace alica
{
class SolverContext;

namespace reasoner
{

class ConstraintTestPlanDummySolver : public ISolver<ConstraintTestPlanDummySolver, int64_t>
{
public:
    ConstraintTestPlanDummySolver(VariableSyncModule* vsm);
    virtual ~ConstraintTestPlanDummySolver();

    bool existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<int64_t>& results);

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
