#include "communication/AlicaRosCommunication.h"

using namespace alica;

namespace alicaRosProxy
{

namespace
{
const std::string planTreeInfoTopic = "/AlicaEngine/PlanTreeInfo";
const std::string solverResultTopic = "/AlicaEngine/SolverResult";
const std::string presenceAnnouncementTopic = "/AlicaEngine/AgentAnnouncement";
const std::string allocationAuthorityInfoTopic = "/AlicaEngine/AllocationAuthorityInfo";
const std::string syncReadyTopic = "/AlicaEngine/SyncReady";
const std::string syncTalkTopic = "/AlicaEngine/SyncTalk";
const std::string presenceQueryTopic = "/AlicaEngine/AgentQuery";

} // namespace

AlicaRosCommunication::AlicaRosCommunication(
        const AlicaCommunicationHandlers& callbacks, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr callbackGroup)
        : AlicaRosCommunicationCommon(callbacks, std::move(node), std::move(callbackGroup))
{
    const auto options = subscriptionOptions();

    _planTreeInfoSubscriber = _node->create_subscription<alica_msgs::msg::PlanTreeInfo>(planTreeInfoTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::PlanTreeInfo>([this](const alica_msgs::msg::PlanTreeInfo& msg) { handlePlanTreeInfoRos(msg); }), options);
    _solverResultSubscriber = _node->create_subscription<alica_msgs::msg::SolverResult>(solverResultTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::SolverResult>([this](const alica_msgs::msg::SolverResult& msg) { handleSolverResult(msg); }), options);
    // Transient local like the publishers, so that an agent that starts later gets the last query and announcement of each
    // agent, as with the latched ROS1 topics
    _presenceAnnouncementSubscriber = _node->create_subscription<alica_msgs::msg::AgentAnnouncement>(presenceAnnouncementTopic,
            subscriptionQos(50).transient_local(),
            guardedCallback<alica_msgs::msg::AgentAnnouncement>([this](const alica_msgs::msg::AgentAnnouncement& msg) { handleAgentAnnouncement(msg); }),
            options);
    _allocationAuthorityInfoSubscriber = _node->create_subscription<alica_msgs::msg::AllocationAuthorityInfo>(allocationAuthorityInfoTopic, subscriptionQos(10),
            guardedCallback<alica_msgs::msg::AllocationAuthorityInfo>(
                    [this](const alica_msgs::msg::AllocationAuthorityInfo& msg) { handleAllocationAuthorityRos(msg); }),
            options);
    _syncReadySubscriber = _node->create_subscription<alica_msgs::msg::SyncReady>(syncReadyTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::SyncReady>([this](const alica_msgs::msg::SyncReady& msg) { handleSyncReadyRos(msg); }), options);
    _syncTalkSubscriber = _node->create_subscription<alica_msgs::msg::SyncTalk>(syncTalkTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::SyncTalk>([this](const alica_msgs::msg::SyncTalk& msg) { handleSyncTalkRos(msg); }), options);
    _presenceQuerySubscriber = _node->create_subscription<alica_msgs::msg::AgentQuery>(presenceQueryTopic, subscriptionQos(5).transient_local(),
            guardedCallback<alica_msgs::msg::AgentQuery>([this](const alica_msgs::msg::AgentQuery& msg) { handleAgentQuery(msg); }), options);

    startSpinning();
}

AlicaRosCommunication::~AlicaRosCommunication()
{
    stopSpinning();
}

} /* namespace alicaRosProxy */
