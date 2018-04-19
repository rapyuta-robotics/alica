#pragma once

#include <AutoDiff.h>
#include <engine/constraintmodul/ISolver.h>
#include <engine/Types.h>
#include <vector>
#include <memory>
#include <mutex>
#include "GSolver.h"

//#define CGSolver_DEBUG

namespace alica {
class AlicaEngine;
class IVariableSyncModule;

namespace reasoner {
class GSolver;

class CGSolver : public ISolver<CGSolver, double> {
public:
    CGSolver(AlicaEngine* ae);
    virtual ~CGSolver();

    bool existsSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
            std::vector<double>& results);

    virtual std::shared_ptr<SolverVariable> createVariable(int64_t id) override;

private:
    GSolver _gs;
    GSolver _sgs;

    std::mutex _mtx;

    double _lastUtil;
    double _lastRuns;
    double _lastFEvals;
};

}  // namespace reasoner
}  // namespace alica
