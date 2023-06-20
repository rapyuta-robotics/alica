#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

#include <atomic>
#include <memory>

namespace ros_utils
{

class WaitForTrigger : public alica::BasicBehaviour
{
public:
    WaitForTrigger(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<WaitForTrigger> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(ros_utils::WaitForTrigger::create, WaitForTrigger)

} /* namespace ros_utils */
