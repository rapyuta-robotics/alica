#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace alica
{
class Go2RandomPosition : public BasicBehaviour
{
public:
    Go2RandomPosition(BehaviourContext& context);
    virtual ~Go2RandomPosition();
    virtual void run();
    static std::unique_ptr<Go2RandomPosition> create(alica::BehaviourContext& context);

protected:
    virtual void initialiseParameters();

private:
};
BOOST_DLL_ALIAS(alica::Go2RandomPosition::create, Go2RandomPosition)
} /* namespace alica */
