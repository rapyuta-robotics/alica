#pragma once

#include <alica_tests/DomainBehaviour.h>
#include <alica_tests/TestWorldModel.h>
#include <boost/dll/alias.hpp>

namespace alica
{
class TriggerC : public DomainBehaviour
{
public:
    TriggerC(BehaviourContext& context);
    virtual ~TriggerC();
    virtual void run();
    static std::unique_ptr<TriggerC> create(alica::BehaviourContext& context);

    int callCounter;
    int initCounter;

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::TriggerC::create, TriggerC)
} /* namespace alica */
