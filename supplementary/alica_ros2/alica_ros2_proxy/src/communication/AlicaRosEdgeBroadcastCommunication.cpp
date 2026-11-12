#include "communication/AlicaRosEdgeBroadcastCommunication.h"

using namespace alica;

namespace alicaRosProxy
{

namespace
{
const std::string planTreeInfoTopic = "/edge_broadcaster/PlanTreeInfos";
const std::string solverResultTopic = "/edge_broadcaster/SolverResults";
const std::string presenceAnnouncementTopic = "/edge_broadcaster/AgentAnnouncements";
const std::string allocationAuthorityInfoTopic = "/edge_broadcaster/AllocationAuthorityInfos";
const std::string syncReadyTopic = "/edge_broadcaster/SyncReadies";
const std::string syncTalkTopic = "/edge_broadcaster/SyncTalks";
const std::string presenceQueryTopic = "/edge_broadcaster/AgentQueries";
} // namespace

AlicaRosEdgeBroadcastCommunication::AlicaRosEdgeBroadcastCommunication(
        const AlicaCommunicationHandlers& callbacks, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr callbackGroup)
        : AlicaRosCommunicationCommon(callbacks, std::move(node), std::move(callbackGroup))
{
    const auto options = subscriptionOptions();

    _planTreeInfoSubscriber = _node->create_subscription<alica_msgs::msg::PlanTreeInfoList>(planTreeInfoTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::PlanTreeInfoList>([this](const alica_msgs::msg::PlanTreeInfoList& msg) { handlePlanTreeInfo(msg); }), options);
    _solverResultSubscriber = _node->create_subscription<alica_msgs::msg::SolverResultList>(solverResultTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::SolverResultList>([this](const alica_msgs::msg::SolverResultList& msg) { handleSolverResult(msg); }), options);
    _presenceAnnouncementSubscriber = _node->create_subscription<alica_msgs::msg::AgentAnnouncementList>(presenceAnnouncementTopic, subscriptionQos(50),
            guardedCallback<alica_msgs::msg::AgentAnnouncementList>(
                    [this](const alica_msgs::msg::AgentAnnouncementList& msg) { handleAgentAnnouncement(msg); }),
            options);
    _allocationAuthorityInfoSubscriber = _node->create_subscription<alica_msgs::msg::AllocationAuthorityInfoList>(allocationAuthorityInfoTopic,
            subscriptionQos(10), guardedCallback<alica_msgs::msg::AllocationAuthorityInfoList>([this](const alica_msgs::msg::AllocationAuthorityInfoList& msg) {
                handleAllocationAuthority(msg);
            }),
            options);
    _syncReadySubscriber = _node->create_subscription<alica_msgs::msg::SyncReadyList>(syncReadyTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::SyncReadyList>([this](const alica_msgs::msg::SyncReadyList& msg) { handleSyncReady(msg); }), options);
    _syncTalkSubscriber = _node->create_subscription<alica_msgs::msg::SyncTalkList>(syncTalkTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::SyncTalkList>([this](const alica_msgs::msg::SyncTalkList& msg) { handleSyncTalk(msg); }), options);
    _presenceQuerySubscriber = _node->create_subscription<alica_msgs::msg::AgentQueryList>(presenceQueryTopic, subscriptionQos(5),
            guardedCallback<alica_msgs::msg::AgentQueryList>([this](const alica_msgs::msg::AgentQueryList& msg) { handleAgentQuery(msg); }), options);

    startSpinning();
}

AlicaRosEdgeBroadcastCommunication::~AlicaRosEdgeBroadcastCommunication()
{
    stopSpinning();
}

void AlicaRosEdgeBroadcastCommunication::handleAgentAnnouncement(const alica_msgs::msg::AgentAnnouncementList& pa)
{
    for (const auto& agent_announcement : pa.agent_announcements) {
        AlicaRosCommunicationCommon::handleAgentAnnouncement(agent_announcement);
    }
}

void AlicaRosEdgeBroadcastCommunication::handlePlanTreeInfo(const alica_msgs::msg::PlanTreeInfoList& pti)
{
    for (const auto& info : pti.plan_tree_infos) {
        AlicaRosCommunicationCommon::handlePlanTreeInfoRos(info);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleSolverResult(const alica_msgs::msg::SolverResultList& sr)
{
    for (const auto& result : sr.solver_results) {
        AlicaRosCommunicationCommon::handleSolverResult(result);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleSyncReady(const alica_msgs::msg::SyncReadyList& sr)
{
    for (const auto& sync_ready : sr.sync_readies) {
        AlicaRosCommunicationCommon::handleSyncReadyRos(sync_ready);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleAllocationAuthority(const alica_msgs::msg::AllocationAuthorityInfoList& aai)
{
    for (const auto& allocation_authority : aai.allocation_authority_infos) {
        AlicaRosCommunicationCommon::handleAllocationAuthorityRos(allocation_authority);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleSyncTalk(const alica_msgs::msg::SyncTalkList& st)
{
    for (const auto& sync_talk : st.sync_talks) {
        AlicaRosCommunicationCommon::handleSyncTalkRos(sync_talk);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleAgentQuery(const alica_msgs::msg::AgentQueryList& pq)
{
    for (const auto& agent_query : pq.agent_queries) {
        AlicaRosCommunicationCommon::handleAgentQuery(agent_query);
    }
}

} /* namespace alicaRosProxy */
