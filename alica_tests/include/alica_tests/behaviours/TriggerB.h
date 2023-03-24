#pragma once

#include <alica_tests/TestWorldModel.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class TriggerB : public BasicBehaviour
{
public:
    TriggerB(BehaviourContext& context);
    virtual ~TriggerB();
    virtual void run();
    static std::unique_ptr<TriggerB> create(alica::BehaviourContext& context);

    int callCounter;
    int initCounter;

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::TriggerB::create, TriggerB)
} /* namespace alica */
