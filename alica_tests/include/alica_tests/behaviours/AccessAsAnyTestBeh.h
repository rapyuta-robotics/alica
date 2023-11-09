#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{

class AccessAsAnyTestBeh : public BasicBehaviour
{
public:
    AccessAsAnyTestBeh(BehaviourContext& context);
    static std::unique_ptr<AccessAsAnyTestBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::AccessAsAnyTestBeh::create, AccessAsAnyTestBeh)
} /* namespace alica */
