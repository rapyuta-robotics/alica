#pragma once
#include <memory>
#include <vector>


namespace alica {
class AlicaEngine;
class Variable;
struct SolverResult;

class IVariableSyncModule {
public:
    virtual ~IVariableSyncModule(){};

    virtual void init() = 0;
    virtual void close() = 0;
    virtual void clear() = 0;
    virtual void onSolverResult(std::shared_ptr<SolverResult> msg) = 0;

    virtual void postResult(long vid, shared_ptr<vector<uint8_t>>& result) = 0;
    virtual shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<uint8_t>>>>>> getSeeds(
            shared_ptr<vector<const Variable*>> query, shared_ptr<vector<shared_ptr<vector<double>>>> limits) = 0;
};
} /* namespace alica */

