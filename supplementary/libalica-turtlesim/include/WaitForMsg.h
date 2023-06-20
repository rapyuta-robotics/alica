#pragma once

#include <engine/BasicBehaviour.h>

#include <boost/dll/alias.hpp>

#include <atomic>
#include <memory>

namespace ros_utils
{

class WaitForMsg : public alica::BasicBehaviour
{
public:
    WaitForMsg(alica::BehaviourContext& context);
    void initialiseParameters() override;
    void run() override;
    static std::unique_ptr<WaitForMsg> create(alica::BehaviourContext& context);
};
BOOST_DLL_ALIAS(ros_utils::WaitForMsg::create, WaitForMsg)

} /* namespace ros_utils */
