#pragma once
#include "alica_msgs/msg/agent_announcement.hpp"
#include "alica_msgs/msg/agent_query.hpp"
#include "alica_msgs/msg/alica_engine_info.hpp"
#include "alica_msgs/msg/allocation_authority_info.hpp"
#include "alica_msgs/msg/plan_tree_info.hpp"
#include "alica_msgs/msg/role_switch.hpp"
#include "alica_msgs/msg/solver_result.hpp"
#include "alica_msgs/msg/sync_ready.hpp"
#include "alica_msgs/msg/sync_talk.hpp"

#include <engine/IAlicaCommunication.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosCommunication : public alica::IAlicaCommunication
{
public:
    AlicaRosCommunication(const AlicaCommunicationHandlers& callbacks);
    virtual ~AlicaRosCommunication();

    void sendAllocationAuthority(const AllocationAuthorityInfo& aai) const override;
    void sendAlicaEngineInfo(const AlicaEngineInfo& bi) const override;
    void sendPlanTreeInfo(const PlanTreeInfo& pti) const override;
    void sendRoleSwitch(const RoleSwitch& rs, AgentId agentID) const override;
    void sendSyncReady(const SyncReady& sr) const override;
    void sendSyncTalk(const SyncTalk& st) const override;
    void sendSolverResult(const SolverResult& sr) const override;
    void sendAgentQuery(const AgentQuery& pq) const override;
    void sendAgentAnnouncement(const AgentAnnouncement& pa) const override;
    void sendLogMessage(int level, const std::string& message) const override;

    void handleAllocationAuthorityRos(const alica_msgs::msg::AllocationAuthorityInfo& aai);
    void handlePlanTreeInfoRos(alica_msgs::msg::PlanTreeInfo pti);
    void handleSyncReadyRos(alica_msgs::msg::SyncReady sr);
    void handleSyncTalkRos(alica_msgs::msg::SyncTalk st);
    void handleSolverResult(const alica_msgs::msg::SolverResult& sr);
    void handleAgentQuery(const alica_msgs::msg::AgentQuery& pq);
    void handleAgentAnnouncement(const alica_msgs::msg::AgentAnnouncement& pa);

    void startCommunication() override;
    void stopCommunication() override;

private:
    rclcpp::Node::SharedPtr _rosNode;

    rclcpp::Publisher<alica_msgs::msg::AlicaEngineInfo>::SharedPtr _alicaEngineInfoPublisher;
    rclcpp::Publisher<alica_msgs::msg::RoleSwitch>::SharedPtr _roleSwitchPublisher;

    rclcpp::Publisher<alica_msgs::msg::AllocationAuthorityInfo>::SharedPtr _allocationAuthorityInfoPublisher;
    rclcpp::Subscription<alica_msgs::msg::AllocationAuthorityInfo>::SharedPtr _allocationAuthorityInfoSubscriber;

    rclcpp::Publisher<alica_msgs::msg::PlanTreeInfo>::SharedPtr _planTreeInfoPublisher;
    rclcpp::Subscription<alica_msgs::msg::PlanTreeInfo>::SharedPtr _planTreeInfoSubscriber;

    rclcpp::Publisher<alica_msgs::msg::SyncReady>::SharedPtr _syncReadyPublisher;
    rclcpp::Subscription<alica_msgs::msg::SyncReady>::SharedPtr _syncReadySubscriber;

    rclcpp::Publisher<alica_msgs::msg::SyncTalk>::SharedPtr _syncTalkPublisher;
    rclcpp::Subscription<alica_msgs::msg::SyncTalk>::SharedPtr _syncTalkSubscriber;

    rclcpp::Publisher<alica_msgs::msg::SolverResult>::SharedPtr _solverResultPublisher;
    rclcpp::Subscription<alica_msgs::msg::SolverResult>::SharedPtr _solverResultSubscriber;

    rclcpp::Publisher<alica_msgs::msg::AgentQuery>::SharedPtr _presenceQueryPublisher;
    rclcpp::Subscription<alica_msgs::msg::AgentQuery>::SharedPtr _presenceQuerySubscriber;

    rclcpp::Publisher<alica_msgs::msg::AgentAnnouncement>::SharedPtr _presenceAnnouncementPublisher;
    rclcpp::Subscription<alica_msgs::msg::AgentAnnouncement>::SharedPtr _presenceAnnouncementSubscriber;

    bool _isRunning;
    rclcpp::executors::MultiThreadedExecutor _myexec;
    std::thread _commThread;
};

} /* namespace alicaRosProxy */
