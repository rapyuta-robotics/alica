#pragma once

#include "GSolver.h"

#include <engine/Types.h>
#include <engine/constraintmodul/ISolver.h>
#include <memory>
#include <mutex>
#include <vector>

//#define CGSolver_DEBUG

namespace alica
{
class AlicaEngine;
class IVariableSyncModule;
class SolverContext;

namespace reasoner
{
class GSolver;

class CGSolver : public ISolver<CGSolver, double>
{
  public:
    CGSolver(AlicaEngine* ae);
    virtual ~CGSolver();

    bool existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<double>& results);

    virtual SolverVariable* createVariable(int64_t id, alica::SolverContext* context) override;
    virtual std::unique_ptr<SolverContext> createSolverContext() override;

  private:
    GSolver _gs;
    GSolver _sgs;

    std::mutex _mtx;

    double _lastUtil;
    double _lastRuns;
    double _lastFEvals;
};

} // namespace reasoner
} // namespace alica
