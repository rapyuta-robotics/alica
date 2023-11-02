#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

#include <chrono>

namespace turtlesim
{

class Teleport : public alica::BasicBehaviour
{
public:
    Teleport(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<Teleport> create(alica::BehaviourContext& context);

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> _startTime;
};
BOOST_DLL_ALIAS(turtlesim::Teleport::create, Teleport)

} /* namespace turtlesim */
