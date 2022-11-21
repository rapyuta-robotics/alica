#pragma once

#include <engine/AlicaClock.h>

#include <rclcpp/node.hpp>

namespace alicaRosProxy
{

class AlicaROSClock : public alica::AlicaClock
{
public:
    AlicaROSClock();
    virtual ~AlicaROSClock() {}
    alica::AlicaTime now() const override;
    void sleep(const alica::AlicaTime&) const override;

private:
    rclcpp::Node::SharedPtr _rosTime;
};

} // namespace alicaRosProxy
