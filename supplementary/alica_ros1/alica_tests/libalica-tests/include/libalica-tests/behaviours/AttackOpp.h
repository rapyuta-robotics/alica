#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class AttackOpp : public DomainBehaviour
{
public:
    AttackOpp(BehaviourContext& context);
    virtual ~AttackOpp();
    virtual void run();
    static std::unique_ptr<AttackOpp> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::AttackOpp::create, AttackOpp)
} /* namespace alica */
