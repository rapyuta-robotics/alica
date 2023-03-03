#pragma once

#include <alica_tests/TestWorldModel.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class TriggerA : public BasicBehaviour
{
public:
    TriggerA(BehaviourContext& context);
    virtual ~TriggerA();
    virtual void run();
    static std::unique_ptr<TriggerA> create(alica::BehaviourContext& context);

    int callCounter;
    int initCounter;

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::TriggerA::create, TriggerA)
} /* namespace alica */
