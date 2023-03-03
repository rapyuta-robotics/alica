#include <alica_tests/behaviours/TriggerA.h>
#include <memory>

namespace alica
{

TriggerA::TriggerA(BehaviourContext& context)
        : BasicBehaviour(context)
{
    this->callCounter = 0;
    this->initCounter = 0;
}
TriggerA::~TriggerA() {}
void TriggerA::run()
{
    callCounter++;
}
void TriggerA::initialiseParameters()
{
    callCounter = 0;
    initCounter++;
}
std::unique_ptr<TriggerA> TriggerA::create(alica::BehaviourContext& context)
{
    return std::make_unique<TriggerA>(context);
}
} /* namespace alica */
