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

#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>

using namespace alica;

namespace alicaRosProxy
{

/**
 * Publishers and message conversion shared by all ROS2 transports.
 *
 * If a node is given, the caller owns the executor that spins it, and the subscriptions use the given callback group
 * (or the node's default group if it is null). If no node is given, an own node is created and spun on an internal
 * single threaded executor, which is the behaviour of the earlier ROS2 proxy.
 *
 * QoS: all topics are reliable. The agent query and agent announcement topics are also transient local on both ends,
 * which replaces the ROS1 latch: an agent gets the last query and announcement of each agent that started before it.
 * A transient local subscription does not match a volatile publisher, so every publisher on these two topics must be
 * transient local.
 *
 * Do not stop or destroy the object from inside one of its subscription callbacks: stopCommunication() and the
 * destructor wait for them.
 */
class AlicaRosCommunicationCommon : public alica::IAlicaCommunication
{
public:
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

    void startCommunication() override;
    void stopCommunication() override;

    void handleAllocationAuthorityRos(const alica_msgs::msg::AllocationAuthorityInfo& aai);
    void handlePlanTreeInfoRos(const alica_msgs::msg::PlanTreeInfo& pti);
    void handleSyncReadyRos(const alica_msgs::msg::SyncReady& sr);
    void handleSyncTalkRos(const alica_msgs::msg::SyncTalk& st);
    void handleSolverResult(const alica_msgs::msg::SolverResult& sr);
    void handleAgentQuery(const alica_msgs::msg::AgentQuery& pq);
    void handleAgentAnnouncement(const alica_msgs::msg::AgentAnnouncement& pa);

protected:
    AlicaRosCommunicationCommon(
            const AlicaCommunicationHandlers& callbacks, rclcpp::Node::SharedPtr node = nullptr, rclcpp::CallbackGroup::SharedPtr callbackGroup = nullptr);
    virtual ~AlicaRosCommunicationCommon();

    // Must be called after the subscriptions are created, so that no message is processed before that
    void startSpinning();
    // Must be called first in the destructor of derived classes, so that no callback runs on a partly destroyed object.
    // Waits for subscription callbacks that are in progress. After it returns, no guarded callback calls its handler.
    void stopSpinning();

    // Wraps a subscription callback for use with stopSpinning(). With a node from the caller, the executor of the caller
    // can still hold a callback when this object is destroyed; the wrapper keeps that callback away from the object.
    template <class MsgT, class HandlerT>
    auto guardedCallback(HandlerT handler) const
    {
        return [guard = _callbackGuard, handler = std::move(handler)](const MsgT& msg) {
            std::shared_lock<std::shared_mutex> lock(guard->mutex);
            if (guard->active) {
                handler(msg);
            }
        };
    }

    rclcpp::SubscriptionOptions subscriptionOptions() const;
    static rclcpp::QoS subscriptionQos(size_t depth);

    rclcpp::Node::SharedPtr _node;
    rclcpp::CallbackGroup::SharedPtr _callbackGroup;

private:
    struct CallbackGuard
    {
        std::shared_mutex mutex;
        bool active{true};
    };
    // Shared with the subscription callbacks, so that it outlives this object
    std::shared_ptr<CallbackGuard> _callbackGuard;

    rclcpp::Publisher<alica_msgs::msg::AlicaEngineInfo>::SharedPtr _alicaEngineInfoPublisher;
    rclcpp::Publisher<alica_msgs::msg::RoleSwitch>::SharedPtr _roleSwitchPublisher;
    rclcpp::Publisher<alica_msgs::msg::AllocationAuthorityInfo>::SharedPtr _allocationAuthorityInfoPublisher;
    rclcpp::Publisher<alica_msgs::msg::PlanTreeInfo>::SharedPtr _planTreeInfoPublisher;
    rclcpp::Publisher<alica_msgs::msg::SyncReady>::SharedPtr _syncReadyPublisher;
    rclcpp::Publisher<alica_msgs::msg::SyncTalk>::SharedPtr _syncTalkPublisher;
    rclcpp::Publisher<alica_msgs::msg::SolverResult>::SharedPtr _solverResultPublisher;
    rclcpp::Publisher<alica_msgs::msg::AgentQuery>::SharedPtr _presenceQueryPublisher;
    rclcpp::Publisher<alica_msgs::msg::AgentAnnouncement>::SharedPtr _presenceAnnouncementPublisher;

    std::atomic<bool> _isRunning;

    // Only used if no node is given
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
    std::thread _spinThread;
};

} /* namespace alicaRosProxy */
