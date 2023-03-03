#include <alica_tests/behaviours/AttackOpp.h>
#include <memory>

namespace alica
{

AttackOpp::AttackOpp(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
AttackOpp::~AttackOpp() {}
void AttackOpp::run() {}
void AttackOpp::initialiseParameters() {}

std::unique_ptr<AttackOpp> AttackOpp::create(alica::BehaviourContext& context)
{
    return std::make_unique<AttackOpp>(context);
}

} /* namespace alica */
