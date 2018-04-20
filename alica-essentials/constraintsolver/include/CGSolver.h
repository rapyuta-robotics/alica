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

class CGSolver : public ISolver {
public:
    CGSolver(AlicaEngine* ae);
    virtual ~CGSolver();

    bool existsSolution(
            const alica::VariableGrp& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls) override;
    bool getSolution(const alica::VariableGrp& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
            std::vector<void*>& results) override;
    std::shared_ptr<SolverVariable> createVariable(long id) override;

protected:
    GSolver _gs;
    GSolver _sgs;

    std::mutex _mtx;

    double _lastUtil;
    double _lastRuns;
    double _lastFEvals;
};

}  // namespace reasoner
}  // namespace alica
