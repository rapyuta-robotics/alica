#pragma once

#include <engine/constraintmodul/ISolver.h>
#include <engine/blackboard/BBIdent.h>
#include <memory>
#include <vector>

namespace alica {

class AlicaEngine;
class ProblemDescriptor;
class Variable;
class SolverVariable;

namespace reasoner {

class DummySolver : public alica::ISolver<DummySolver, BBIdent> {
public:
    DummySolver(AlicaEngine* ae);
    virtual ~DummySolver();

    bool existsSolutionImpl(const VariableGrp& vars, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
    bool getSolutionImpl(const VariableGrp&, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
            std::vector<BBIdent>& results);
    virtual std::shared_ptr<SolverVariable> createVariable(int64_t representingVariableId) override;
};
} /* namespace reasoner */
} /* namespace alica */
