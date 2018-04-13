#pragma once

#include <engine/constraintmodul/ISolver.h>
#include <vector>
#include <memory>

namespace alica {
namespace reasoner {

class ConstraintTestPlanDummySolver : public ISolver {
public:
    ConstraintTestPlanDummySolver(AlicaEngine* ae);
    virtual ~ConstraintTestPlanDummySolver();

    bool existsSolution(const VariableSet& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls) override;
    bool getSolution(const VariableSet& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
            std::vector<void*>& results) override;
    std::shared_ptr<SolverVariable> createVariable(long id) override;

    static int getExistsSolutionCallCounter();
    static int getGetSolutionCallCounter();

private:
    static int existsSolutionCallCounter;
    static int getSolutionCallCounter;
};
} /* namespace reasoner */

} /* namespace alica */
