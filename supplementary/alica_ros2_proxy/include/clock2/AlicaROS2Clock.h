#pragma once

#include <engine/AlicaClock.h>
#include <rclcpp/node.hpp>

namespace alicaRos2Proxy
{

class AlicaROS2Clock : public alica::AlicaClock
{
public:
    AlicaROS2Clock();
    virtual ~AlicaROS2Clock() {}
    alica::AlicaTime now() const override;
    void sleep(const alica::AlicaTime&) const override;

private:
    std::shared_ptr<rclcpp::Node> _rosTime;
};

} // namespace alicaRos2Proxy
