#include <libalica-tests/behaviours/NotToTrigger.h>
#include <memory>

namespace alica
{

NotToTrigger::NotToTrigger(BehaviourContext& context)
        : DomainBehaviour(context)
{
    this->callCounter = 0;
    this->initCounter = 0;
}
NotToTrigger::~NotToTrigger() {}
void NotToTrigger::run()
{
    callCounter++;
}
void NotToTrigger::initialiseParameters()
{
    callCounter = 0;
    initCounter++;
}

std::unique_ptr<NotToTrigger> NotToTrigger::create(alica::BehaviourContext& context)
{
    return std::make_unique<NotToTrigger>(context);
}

} /* namespace alica */
