#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

namespace turtlesim
{

class Teleport2RandomPosition : public alica::BasicBehaviour
{
public:
    Teleport2RandomPosition(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<Teleport2RandomPosition> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(turtlesim::Teleport2RandomPosition::create, Teleport2RandomPosition)

} /* namespace turtlesim */
