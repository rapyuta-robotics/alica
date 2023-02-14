#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <alica_tests/TestWorldModel.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class TriggerA : public DomainBehaviour
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
