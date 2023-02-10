#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class Attack : public DomainBehaviour
{
public:
    Attack(BehaviourContext& context);
    virtual ~Attack();
    virtual void run();
    static std::unique_ptr<Attack> create(alica::BehaviourContext& context);

    int callCounter;
    int initCounter;

protected:
    virtual void initialiseParameters();

private:
};
BOOST_DLL_ALIAS(alica::Attack::create, Attack)
} /* namespace alica */
