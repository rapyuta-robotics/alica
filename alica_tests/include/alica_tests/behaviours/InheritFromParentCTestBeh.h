#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class InheritFromParentCTestBeh : public BasicBehaviour
{
public:
    InheritFromParentCTestBeh(BehaviourContext& context);
    static std::unique_ptr<InheritFromParentCTestBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::InheritFromParentCTestBeh::create, InheritFromParentCTestBeh)
} /* namespace alica */
