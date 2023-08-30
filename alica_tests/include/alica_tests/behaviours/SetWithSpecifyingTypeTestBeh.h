#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{

class SetWithSpecifyingTypeTestBeh : public BasicBehaviour
{
public:
    SetWithSpecifyingTypeTestBeh(BehaviourContext& context);
    static std::unique_ptr<SetWithSpecifyingTypeTestBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::SetWithSpecifyingTypeTestBeh::create, SetWithSpecifyingTypeTestBeh)
} /* namespace alica */
