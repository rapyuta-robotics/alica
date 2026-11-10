#pragma once

#include "communication/AlicaRosCommunicationCommon.h"

#include <rclcpp/rclcpp.hpp>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosCommunication : public AlicaRosCommunicationCommon
{
public:
    AlicaRosCommunication(
            const AlicaCommunicationHandlers& callbacks, rclcpp::Node::SharedPtr node = nullptr, rclcpp::CallbackGroup::SharedPtr callbackGroup = nullptr);
    virtual ~AlicaRosCommunication();

private:
    rclcpp::Subscription<alica_msgs::msg::SyncReady>::SharedPtr _syncReadySubscriber;
    rclcpp::Subscription<alica_msgs::msg::PlanTreeInfo>::SharedPtr _planTreeInfoSubscriber;
    rclcpp::Subscription<alica_msgs::msg::SolverResult>::SharedPtr _solverResultSubscriber;
    rclcpp::Subscription<alica_msgs::msg::AgentAnnouncement>::SharedPtr _presenceAnnouncementSubscriber;
    rclcpp::Subscription<alica_msgs::msg::AllocationAuthorityInfo>::SharedPtr _allocationAuthorityInfoSubscriber;
    rclcpp::Subscription<alica_msgs::msg::SyncTalk>::SharedPtr _syncTalkSubscriber;
    rclcpp::Subscription<alica_msgs::msg::AgentQuery>::SharedPtr _presenceQuerySubscriber;
};

} /* namespace alicaRosProxy */
