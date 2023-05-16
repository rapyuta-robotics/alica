#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class AddToValueBeh : public BasicBehaviour
{
public:
    AddToValueBeh(BehaviourContext& context);
    static std::unique_ptr<AddToValueBeh> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::AddToValueBeh::create, AddToValueBeh)
} /* namespace alica */
