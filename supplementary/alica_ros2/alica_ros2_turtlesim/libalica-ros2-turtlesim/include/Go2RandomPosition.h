#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicBehaviour.h>

namespace turtlesim
{
class Go2RandomPosition : public alica::BasicBehaviour
{
public:
    Go2RandomPosition(alica::BehaviourContext& context);
    virtual ~Go2RandomPosition();
    virtual void run();
    static std::unique_ptr<Go2RandomPosition> create(alica::BehaviourContext& context);

private:
};
BOOST_DLL_ALIAS(turtlesim::Go2RandomPosition::create, Go2RandomPosition)
} /* namespace turtlesim */
