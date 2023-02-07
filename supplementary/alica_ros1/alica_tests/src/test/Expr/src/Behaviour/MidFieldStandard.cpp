#include <alica_tests/Behaviour/MidFieldStandard.h>
#include <memory>

namespace alica
{

MidFieldStandard::MidFieldStandard(BehaviourContext& context)
        : DomainBehaviour(context)
{
    this->callCounter = 0;
}
MidFieldStandard::~MidFieldStandard() {}
void MidFieldStandard::run()
{
    callCounter++;
    if (callCounter > 10) {
        this->setSuccess();
    }
}
void MidFieldStandard::initialiseParameters()
{
    this->callCounter = 0;
}
std::unique_ptr<MidFieldStandard> MidFieldStandard::create(alica::BehaviourContext& context)
{
    return std::make_unique<MidFieldStandard>(context);
}

} /* namespace alica */
