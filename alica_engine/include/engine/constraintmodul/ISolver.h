#pragma once

#include <memory>
#include <vector>

namespace alica {
class AlicaEngine;
class ProblemDescriptor;
class Variable;
class SolverVariable;

class ISolver : public std::enable_shared_from_this<ISolver> {
public:
    ISolver(AlicaEngine* ae) {
        this->ae = ae;
    }
    virtual ~ISolver() {}

    virtual bool existsSolution(
            std::vector<Variable*>& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls) = 0;
    virtual bool getSolution(std::vector<Variable*>& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
            std::vector<void*>& results) = 0;
    virtual std::shared_ptr<SolverVariable> createVariable(long id) = 0;

    virtual double utilityEstimate(
            std::vector<Variable*>& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls) {
        return 0;
    }

    AlicaEngine* getAlicaEngine() {
        return ae;
    }

protected:
    AlicaEngine* ae;
};

} /* namespace alica */
