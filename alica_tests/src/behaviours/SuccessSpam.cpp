#include <alica_tests/behaviours/SuccessSpam.h>
#include <memory>

/*PROTECTED REGION ID(inccpp1522377401286) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{

SuccessSpam::SuccessSpam(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
SuccessSpam::~SuccessSpam() {}
void SuccessSpam::run()
{
    setSuccess();
}
void SuccessSpam::initialiseParameters() {}
std::unique_ptr<SuccessSpam> SuccessSpam::create(alica::BehaviourContext& context)
{
    return std::make_unique<SuccessSpam>(context);
}
} /* namespace alica */
