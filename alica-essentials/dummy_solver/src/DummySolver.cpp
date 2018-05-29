#include "alica/reasoner/DummySolver.h"
#include "alica/reasoner/DummyContext.h"
#include "alica/reasoner/DummyTerm.h"
#include "alica/reasoner/DummyVariable.h"

#include <engine/AlicaEngine.h>
#include <engine/blackboard/BlackBoard.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Variable.h>

namespace alica
{

namespace reasoner
{

DummySolver::DummySolver(AlicaEngine* ae)
    : ISolver(ae)
{
}

DummySolver::~DummySolver() {}

bool DummySolver::existsSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls)
{
    return true;
}

bool DummySolver::getSolutionImpl(SolverContext* ctx, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls, std::vector<BBIdent>& results)
{

    DummyContext* dc = static_cast<DummyContext*>(ctx);

    results.reserve(dc->getVariables().size());

    BlackBoard& bb = getAlicaEngine()->editBlackBoard();
    for (const std::unique_ptr<DummyVariable>& dummyVariable : dc->getVariables()) {
        const std::string& val = getValue(dummyVariable->getId(), calls);
        BBIdent bid = bb.registerValue(val.c_str(), val.size());
        results.push_back(bid);
    }

    return true;
}

SolverVariable* DummySolver::createVariable(int64_t representingVariableID, SolverContext* ctx)
{
    return static_cast<DummyContext*>(ctx)->createVariable(representingVariableID);
}
std::unique_ptr<SolverContext> DummySolver::createSolverContext()
{
    return std::unique_ptr<SolverContext>(new DummyContext());
}

const std::string& DummySolver::getValue(int64_t id, const std::vector<std::shared_ptr<ProblemDescriptor>>& calls) const
{
    for (const auto& c : calls) {
        DummyTerm* constraintTerm = dynamic_cast<DummyTerm*>(c->getConstraint());
        if (!constraintTerm) {
            std::cerr << "DummySolver: Constraint type not compatible with selected solver!" << std::endl;
            continue;
        }
        const std::string* value = constraintTerm->tryGetValue(id);
        if (value) {
            return *value;
        }
    }
    return DummyVariable::NO_VALUE;
}

} /* namespace reasoner */
} /* namespace alica */
