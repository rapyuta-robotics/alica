#pragma once
#include <memory>
#include <vector>
#include "engine/collections/Variant.h"
#include "engine/Types.h"

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
    virtual void onSolverResult(const SolverResult& msg) = 0;

    virtual void postResult(int64_t vid, Variant result) = 0;
    virtual int getSeeds(const VariableSet& query, const std::vector<double>& limits, std::vector<Variant>& o_seeds) const = 0;
};
} /* namespace alica */

