#pragma once

#include "engine/BasicBehaviour.h"
#include <boost/dll/alias.hpp>

namespace alica
{
class Go2RandomPosition : public BasicBehaviour
{
public:
    Go2RandomPosition(BehaviourContext& context);
    virtual ~Go2RandomPosition();
    virtual void run();
    // Factory method
    static std::unique_ptr<Go2RandomPosition> create(BehaviourContext& context) { return std::unique_ptr<Go2RandomPosition>(new Go2RandomPosition(context)); }

protected:
    virtual void initialiseParameters();

private:
};
BOOST_DLL_ALIAS(alica::Go2RandomPosition::create, Go2RandomPosition)
} /* namespace alica */
