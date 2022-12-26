#include "alica/reasoner/SimpleSolver.h"
#include "alica/reasoner/SimpleTerm.h"
#include "alica/reasoner/SimpleVariable.h"
#include "alica_solver_interface/SimpleContext.h"

#include <engine/AlicaEngine.h>
#include <engine/blackboard/Blackboard.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Variable.h>

namespace alica
{

namespace reasoner
{

SimpleSolver::SimpleSolver(AlicaEngine* ae)
        : ISolver(ae)
{
}

SimpleSolver::~SimpleSolver() {}

bool SimpleSolver::existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls)
{
    return true;
}

bool SimpleSolver::getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<int64_t>& results)
{
    SimpleContext<SimpleVariable>* dc = static_cast<SimpleContext<SimpleVariable>*>(ctx);

    results.reserve(dc->getVariables().size());

    Blackboard& bb = getAlicaEngine()->editGlobalBlackboard();
    for (const std::unique_ptr<SimpleVariable>& dummyVariable : dc->getVariables()) {
        const std::string& val = getValue(dummyVariable->getId(), calls);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        LockedBlackboardRW(bb).registerValue(val, dummyVariable->getId());
#pragma GCC diagnostic pop
        results.push_back(dummyVariable->getId());
    }

    return true;
}

SolverVariable* SimpleSolver::createVariable(int64_t representingVariableID, SolverContext* ctx)
{
    return static_cast<SimpleContext<SimpleVariable>*>(ctx)->createVariable(representingVariableID);
}
std::unique_ptr<SolverContext> SimpleSolver::createSolverContext()
{
    return std::unique_ptr<SolverContext>(new SimpleContext<int64_t>());
}

const std::string& SimpleSolver::getValue(int64_t id, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls) const
{
    for (const auto& c : calls) {
        SimpleTerm* constraintTerm = dynamic_cast<SimpleTerm*>(c->getConstraint());
        if (!constraintTerm) {
            std::cerr << "SimpleSolver: Constraint type not compatible with selected solver!" << std::endl;
            continue;
        }
        const std::string* value = constraintTerm->tryGetValue(id);
        if (value) {
            return *value;
        }
    }
    return SimpleVariable::NO_VALUE;
}

} /* namespace reasoner */
} /* namespace alica */
