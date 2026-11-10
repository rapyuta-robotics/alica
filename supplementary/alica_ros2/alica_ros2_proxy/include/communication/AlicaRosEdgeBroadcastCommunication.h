#pragma once
#include "alica_msgs/msg/agent_announcement_list.hpp"
#include "alica_msgs/msg/agent_query_list.hpp"
#include "alica_msgs/msg/allocation_authority_info_list.hpp"
#include "alica_msgs/msg/plan_tree_info_list.hpp"
#include "alica_msgs/msg/solver_result_list.hpp"
#include "alica_msgs/msg/sync_ready_list.hpp"
#include "alica_msgs/msg/sync_talk_list.hpp"

#include "communication/AlicaRosCommunicationCommon.h"

#include <rclcpp/rclcpp.hpp>

using namespace alica;

namespace alicaRosProxy
{

/**
 * Publishes like AlicaRosCommunication, but receives the batched *List messages that the edge broadcaster sends on
 * the /edge_broadcaster topics.
 */
class AlicaRosEdgeBroadcastCommunication : public AlicaRosCommunicationCommon
{
public:
    AlicaRosEdgeBroadcastCommunication(
            const AlicaCommunicationHandlers& callbacks, rclcpp::Node::SharedPtr node = nullptr, rclcpp::CallbackGroup::SharedPtr callbackGroup = nullptr);
    virtual ~AlicaRosEdgeBroadcastCommunication();

private:
    void handleSyncReady(const alica_msgs::msg::SyncReadyList& sr);
    void handlePlanTreeInfo(const alica_msgs::msg::PlanTreeInfoList& pti);
    void handleSolverResult(const alica_msgs::msg::SolverResultList& sr);
    void handleAgentAnnouncement(const alica_msgs::msg::AgentAnnouncementList& pa);
    void handleAllocationAuthority(const alica_msgs::msg::AllocationAuthorityInfoList& aai);
    void handleSyncTalk(const alica_msgs::msg::SyncTalkList& st);
    void handleAgentQuery(const alica_msgs::msg::AgentQueryList& pq);

    rclcpp::Subscription<alica_msgs::msg::SyncReadyList>::SharedPtr _syncReadySubscriber;
    rclcpp::Subscription<alica_msgs::msg::PlanTreeInfoList>::SharedPtr _planTreeInfoSubscriber;
    rclcpp::Subscription<alica_msgs::msg::SolverResultList>::SharedPtr _solverResultSubscriber;
    rclcpp::Subscription<alica_msgs::msg::AgentAnnouncementList>::SharedPtr _presenceAnnouncementSubscriber;
    rclcpp::Subscription<alica_msgs::msg::AllocationAuthorityInfoList>::SharedPtr _allocationAuthorityInfoSubscriber;
    rclcpp::Subscription<alica_msgs::msg::SyncTalkList>::SharedPtr _syncTalkSubscriber;
    rclcpp::Subscription<alica_msgs::msg::AgentQueryList>::SharedPtr _presenceQuerySubscriber;
};

} /* namespace alicaRosProxy */
