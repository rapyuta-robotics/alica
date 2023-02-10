#include <alica_tests/Behaviour/SuccessOnInitBeh.h>
#include <memory>

namespace alica
{

SuccessOnInitBeh::SuccessOnInitBeh(BehaviourContext& context)
        : DomainBehaviour(context)
{
}
SuccessOnInitBeh::~SuccessOnInitBeh() {}
void SuccessOnInitBeh::run() {}
void SuccessOnInitBeh::initialiseParameters()
{
    setSuccess();
}
std::unique_ptr<SuccessOnInitBeh> SuccessOnInitBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<SuccessOnInitBeh>(context);
}
} /* namespace alica */
