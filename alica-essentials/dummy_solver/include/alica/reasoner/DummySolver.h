#pragma once

#include <engine/constraintmodul/ISolver.h>

#include <memory>
#include <vector>

namespace alica {

class AlicaEngine;
class ProblemDescriptor;
class Variable;
class SolverVariable;

namespace reasoner {

class DummySolver : public alica::ISolver<DummySolver, int64_t> {
public:
    DummySolver(AlicaEngine* ae);
    virtual ~DummySolver();

    bool existsSolutionImpl(const VariableSet& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(const VariableSet&, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<int64_t>& results);
    virtual std::shared_ptr<SolverVariable> createVariable(int64_t representingVariableId) override;
};
} /* namespace reasoner */
} /* namespace alica */
