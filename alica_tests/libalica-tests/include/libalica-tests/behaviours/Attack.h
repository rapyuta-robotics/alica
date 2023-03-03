#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class Attack : public BasicBehaviour
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
