#include <libalica-tests/behaviours/TriggerC.h>
#include <memory>

namespace alica
{

TriggerC::TriggerC(BehaviourContext& context)
        : BasicBehaviour(context)
{
    this->callCounter = 0;
    this->initCounter = 0;
}
TriggerC::~TriggerC() {}
void TriggerC::run()
{
    callCounter++;
}
void TriggerC::initialiseParameters()
{

    callCounter = 0;
    initCounter++;
}
std::unique_ptr<TriggerC> TriggerC::create(alica::BehaviourContext& context)
{
    return std::make_unique<TriggerC>(context);
}
} /* namespace alica */
