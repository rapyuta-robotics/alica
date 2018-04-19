#pragma once

#include <engine/constraintmodul/ISolver.h>
#include <engine/blackboard/BBIdent.h>
#include <vector>
#include <memory>

namespace alica {
namespace reasoner {

class ConstraintTestPlanDummySolver : public ISolver<ConstraintTestPlanDummySolver, BBIdent> {
public:
    ConstraintTestPlanDummySolver(AlicaEngine* ae);
    virtual ~ConstraintTestPlanDummySolver();

    bool existsSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
            std::vector<BBIdent>& results);
    std::shared_ptr<SolverVariable> createVariable(int64_t id) override;

    static int getExistsSolutionCallCounter();
    static int getGetSolutionCallCounter();

private:
    static int existsSolutionCallCounter;
    static int getSolutionCallCounter;
};
} /* namespace reasoner */

} /* namespace alica */
