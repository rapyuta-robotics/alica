#include <alica_tests/Behaviour/DefendMid.h>
#include <memory>

namespace alica
{

DefendMid::DefendMid(BehaviourContext& context)
        : DomainBehaviour(context)
{
}
DefendMid::~DefendMid() {}
void DefendMid::run() {}
void DefendMid::initialiseParameters() {}

std::unique_ptr<DefendMid> DefendMid::create(alica::BehaviourContext& context)
{
    return std::make_unique<DefendMid>(context);
}

} /* namespace alica */
