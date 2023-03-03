#include <alica_tests/behaviours/ConstraintUsingBehaviour.h>
#include <memory>

#include <alica_tests/ConstraintTestPlanDummySolver.h>
#include <iostream>

namespace alica
{
std::vector<int64_t> ConstraintUsingBehaviour::result;

ConstraintUsingBehaviour::ConstraintUsingBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
    _callCounter = 0;
}
ConstraintUsingBehaviour::~ConstraintUsingBehaviour() {}
void ConstraintUsingBehaviour::run()
{
    ++_callCounter;
    if (result.size() > 0) {
        return;
    }
    _query.getSolution<reasoner::ConstraintTestPlanDummySolver, int64_t>(getPlanContext(), result);
}
void ConstraintUsingBehaviour::initialiseParameters()
{
    _callCounter = 0;
    _query.clearStaticVariables();
    _query.addStaticVariable(getVariable("Y"));
}
int ConstraintUsingBehaviour::getCallCounter() const
{
    return _callCounter;
}

std::unique_ptr<ConstraintUsingBehaviour> ConstraintUsingBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<ConstraintUsingBehaviour>(context);
}
} /* namespace alica */
