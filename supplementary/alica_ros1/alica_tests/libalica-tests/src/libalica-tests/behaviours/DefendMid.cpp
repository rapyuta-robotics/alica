#include <libalica-tests/behaviours/DefendMid.h>
#include <memory>

namespace alica
{

DefendMid::DefendMid(BehaviourContext& context)
        : BasicBehaviour(context)
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
