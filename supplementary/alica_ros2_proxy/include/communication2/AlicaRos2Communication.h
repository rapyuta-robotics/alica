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

// #include <ros/callback_queue.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosCommunication : public alica::IAlicaCommunication
{
public:
    AlicaRosCommunication(const AlicaCommunicationHandlers& callbacks);
    virtual ~AlicaRosCommunication();

    void tick();

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
    std::shared_ptr<rclcpp::Node> _rosNode;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::AlicaEngineInfo>> _alicaEngineInfoPublisher;
    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::RoleSwitch>> _roleSwitchPublisher;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::AllocationAuthorityInfo>> _allocationAuthorityInfoPublisher;
    std::shared_ptr<rclcpp::Subscription<alica_msgs::msg::AllocationAuthorityInfo>> _allocationAuthorityInfoSubscriber;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::PlanTreeInfo>> _planTreeInfoPublisher;
    std::shared_ptr<rclcpp::Subscription<alica_msgs::msg::PlanTreeInfo>> _planTreeInfoSubscriber;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::SyncReady>> _syncReadyPublisher;
    std::shared_ptr<rclcpp::Subscription<alica_msgs::msg::SyncReady>> _syncReadySubscriber;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::SyncTalk>> _syncTalkPublisher;
    std::shared_ptr<rclcpp::Subscription<alica_msgs::msg::SyncTalk>> _syncTalkSubscriber;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::SolverResult>> _solverResultPublisher;
    std::shared_ptr<rclcpp::Subscription<alica_msgs::msg::SolverResult>> _solverResultSubscriber;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::AgentQuery>> _presenceQueryPublisher;
    std::shared_ptr<rclcpp::Subscription<alica_msgs::msg::AgentQuery>> _presenceQuerySubscriber;

    std::shared_ptr<rclcpp::Publisher<alica_msgs::msg::AgentAnnouncement>> _presenceAnnouncementPublisher;
    std::shared_ptr<rclcpp::Subscription<alica_msgs::msg::AgentAnnouncement>> _presenceAnnouncementSubscriber;

    bool _isRunning;
    rclcpp::executors::MultiThreadedExecutor _myexec;
    std::thread _commThread;
};

} /* namespace alicaRosProxy */
