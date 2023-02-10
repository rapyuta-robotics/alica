#include <alica_tests/Behaviour/TriggerB.h>
#include <memory>

namespace alica
{

TriggerB::TriggerB(BehaviourContext& context)
        : DomainBehaviour(context)
{
    this->callCounter = 0;
    this->initCounter = 0;
}
TriggerB::~TriggerB() {}
void TriggerB::run()
{
    callCounter++;
}
void TriggerB::initialiseParameters()
{
    callCounter = 0;
    initCounter++;
}
std::unique_ptr<TriggerB> TriggerB::create(alica::BehaviourContext& context)
{
    return std::make_unique<TriggerB>(context);
}

} /* namespace alica */
