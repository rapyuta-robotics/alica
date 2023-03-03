#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class AttackOpp : public BasicBehaviour
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
