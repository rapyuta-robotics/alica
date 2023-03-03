#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class CountIndefinitely : public BasicBehaviour
{
public:
    CountIndefinitely(BehaviourContext& context);
    virtual ~CountIndefinitely();
    virtual void run();
    static std::unique_ptr<CountIndefinitely> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();
};
BOOST_DLL_ALIAS(alica::CountIndefinitely::create, CountIndefinitely)
} /* namespace alica */
