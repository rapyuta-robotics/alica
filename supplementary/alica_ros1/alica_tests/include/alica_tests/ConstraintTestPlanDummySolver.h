#pragma once

#include <engine/constraintmodul/ISolver.h>
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
    ConstraintTestPlanDummySolver(AlicaEngine* ae);
    virtual ~ConstraintTestPlanDummySolver();

    bool existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<int64_t>& results);

    SolverVariable* createVariable(int64_t id, SolverContext* ctx) override;
    std::unique_ptr<SolverContext> createSolverContext() override;

    static int getExistsSolutionCallCounter();
    static int getGetSolutionCallCounter();

private:
    static int s_existsSolutionCallCounter;
    static int s_getSolutionCallCounter;
};
} /* namespace reasoner */

} /* namespace alica */
