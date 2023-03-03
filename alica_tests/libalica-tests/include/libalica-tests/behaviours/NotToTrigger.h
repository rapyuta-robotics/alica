#pragma once

#include <alica_tests/TestWorldModel.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class NotToTrigger : public BasicBehaviour
{
public:
    NotToTrigger(BehaviourContext& context);
    virtual ~NotToTrigger();
    virtual void run();
    static std::unique_ptr<NotToTrigger> create(alica::BehaviourContext& context);

    int callCounter;
    int initCounter;

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::NotToTrigger::create, NotToTrigger)
} /* namespace alica */
