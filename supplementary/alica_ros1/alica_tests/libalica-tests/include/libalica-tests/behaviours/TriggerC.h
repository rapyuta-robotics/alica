#pragma once

#include <alica_tests/TestWorldModel.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class TriggerC : public BasicBehaviour
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
