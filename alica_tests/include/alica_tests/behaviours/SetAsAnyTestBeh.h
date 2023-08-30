#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{

class SetAsAnyTestBeh : public BasicBehaviour
{
public:
    SetAsAnyTestBeh(BehaviourContext& context);
    static std::unique_ptr<SetAsAnyTestBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::SetAsAnyTestBeh::create, SetAsAnyTestBeh)
} /* namespace alica */
