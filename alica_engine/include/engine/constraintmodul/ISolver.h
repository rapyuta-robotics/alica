#pragma once

#include "engine/Types.h"
#include <memory>

namespace alica
{
class AlicaEngine;
class ProblemDescriptor;
class Variable;
class SolverVariable;
class SolverContext;
class Blackboard;
class VariableSyncModule;

class ISolverBase
{
public:
    ISolverBase(Blackboard& blackboard, const VariableSyncModule& resultStore, const YAML::Node& config)
            : _blackboard(blackboard)
            , _resultStore(resultStore)
            , _config(config)
    {
    }
    virtual ~ISolverBase() {}
    virtual SolverVariable* createVariable(int64_t id, SolverContext* ctx) = 0;
    virtual std::unique_ptr<SolverContext> createSolverContext() = 0;

protected:
    Blackboard& editBlackboard() { return _blackboard; };
    const VariableSyncModule& getResultStore() { return _resultStore; };
    const YAML::Node& getConfig() const { return _config; };

private:
    Blackboard& _blackboard;
    const VariableSyncModule& _resultStore;
    const YAML::Node& _config;
};

template <class SolverType, typename ResultType>
class ISolver : public ISolverBase
{
public:
    ISolver(Blackboard& blackboard, const VariableSyncModule& resultStore, const YAML::Node& config)
            : ISolverBase(blackboard, resultStore, config)
    {
    }
    virtual ~ISolver() {}

    bool existsSolution(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls)
    {
        return static_cast<SolverType*>(this)->existsSolutionImpl(ctx, calls);
    }

    bool getSolution(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<ResultType>& results)
    {
        return static_cast<SolverType*>(this)->getSolutionImpl(ctx, calls, results);
    }
};

} /* namespace alica */
