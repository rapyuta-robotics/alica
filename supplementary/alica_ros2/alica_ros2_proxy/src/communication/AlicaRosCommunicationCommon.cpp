#include "communication/AlicaRosCommunicationCommon.h"

#include <engine/AlicaEngine.h>
#include <engine/containers/AgentQuery.h>
#include <engine/containers/AlicaEngineInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/containers/RoleSwitch.h>
#include <engine/containers/SolverResult.h>
#include <engine/containers/SolverVar.h>
#include <engine/containers/SyncData.h>
#include <engine/containers/SyncReady.h>
#include <engine/containers/SyncTalk.h>
#include <engine/teammanager/TeamManager.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

#include <unistd.h>

using namespace alica;

namespace alicaRosProxy
{

using std::make_shared;
using std::string;

namespace
{
const std::string allocationAuthorityInfoTopic = "/AlicaEngine/AllocationAuthorityInfo";
const std::string alicaEngineInfoTopic = "/AlicaEngine/AlicaEngineInfo";
const std::string ownRoleTopic = "/AlicaEngine/OwnRole";
const std::string planTreeInfoTopic = "/AlicaEngine/PlanTreeInfo";
const std::string syncReadyTopic = "/AlicaEngine/SyncReady";
const std::string syncTalkTopic = "/AlicaEngine/SyncTalk";
const std::string solverResultTopic = "/AlicaEngine/SolverResult";
const std::string presenceQueryTopic = "/AlicaEngine/AgentQuery";
const std::string presenceAnnouncementTopic = "/AlicaEngine/AgentAnnouncement";

rclcpp::QoS reliableQos(size_t depth)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable();
}
} // namespace

AlicaRosCommunicationCommon::AlicaRosCommunicationCommon(
        const AlicaCommunicationHandlers& callbacks, rclcpp::Node::SharedPtr node, rclcpp::CallbackGroup::SharedPtr callbackGroup)
        : IAlicaCommunication(callbacks)
        , _node(std::move(node))
        , _callbackGroup(std::move(callbackGroup))
        , _callbackGuard(std::make_shared<CallbackGuard>())
        , _isRunning(false)
{
    if (!_node) {
        // A unique name, so that more than one engine can run in a process
        static std::atomic<int> instanceCount(0);
        _node = rclcpp::Node::make_shared("alica_communication_" + std::to_string(getpid()) + "_" + std::to_string(instanceCount++));
        _executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    }

    _allocationAuthorityInfoPublisher = _node->create_publisher<alica_msgs::msg::AllocationAuthorityInfo>(allocationAuthorityInfoTopic, reliableQos(2));
    _alicaEngineInfoPublisher = _node->create_publisher<alica_msgs::msg::AlicaEngineInfo>(alicaEngineInfoTopic, reliableQos(2));
    _roleSwitchPublisher = _node->create_publisher<alica_msgs::msg::RoleSwitch>(ownRoleTopic, reliableQos(10));
    _planTreeInfoPublisher = _node->create_publisher<alica_msgs::msg::PlanTreeInfo>(planTreeInfoTopic, reliableQos(10));
    _syncReadyPublisher = _node->create_publisher<alica_msgs::msg::SyncReady>(syncReadyTopic, reliableQos(10));
    _syncTalkPublisher = _node->create_publisher<alica_msgs::msg::SyncTalk>(syncTalkTopic, reliableQos(10));
    _solverResultPublisher = _node->create_publisher<alica_msgs::msg::SolverResult>(solverResultTopic, reliableQos(10));
    // Latched in ROS1, so that agents that start later still get the last query and announcement
    _presenceQueryPublisher = _node->create_publisher<alica_msgs::msg::AgentQuery>(presenceQueryTopic, reliableQos(5).transient_local());
    _presenceAnnouncementPublisher = _node->create_publisher<alica_msgs::msg::AgentAnnouncement>(presenceAnnouncementTopic, reliableQos(5).transient_local());
}

AlicaRosCommunicationCommon::~AlicaRosCommunicationCommon()
{
    stopSpinning();
}

void AlicaRosCommunicationCommon::startSpinning()
{
    if (_executor && !_spinThread.joinable()) {
        _executor->add_node(_node);
        _spinThread = std::thread([this]() { _executor->spin(); });
    }
}

void AlicaRosCommunicationCommon::stopSpinning()
{
    if (_spinThread.joinable()) {
        _executor->cancel();
        _spinThread.join();
        _executor->remove_node(_node);
    }
    // Waits for callbacks that the executor of the caller is running, and stops later ones
    std::unique_lock<std::shared_mutex> lock(_callbackGuard->mutex);
    _callbackGuard->active = false;
}

rclcpp::SubscriptionOptions AlicaRosCommunicationCommon::subscriptionOptions() const
{
    rclcpp::SubscriptionOptions options;
    options.callback_group = _callbackGroup;
    return options;
}

rclcpp::QoS AlicaRosCommunicationCommon::subscriptionQos(size_t depth)
{
    // Volatile by default, so that the subscription also matches publishers that are not transient local, for example the edge broadcaster
    return reliableQos(depth);
}

void AlicaRosCommunicationCommon::sendAllocationAuthority(const AllocationAuthorityInfo& aai) const
{
    alica_msgs::msg::AllocationAuthorityInfo aais;

    aais.plan_id = aai.planId;
    aais.parent_state = aai.parentState;
    aais.plan_type = aai.planType;

    aais.sender_id = aai.senderID;
    aais.authority = aai.authority;

    for (auto& ep : aai.entryPointRobots) {
        alica_msgs::msg::EntryPointRobots newEP;
        newEP.entry_point = ep.entrypoint;
        for (auto& agentId : ep.robots) {
            newEP.robots.push_back(agentId);
        }
        aais.entrypoints.push_back(newEP);
    }

    if (_isRunning) {
        _allocationAuthorityInfoPublisher->publish(aais);
    }
}

void AlicaRosCommunicationCommon::sendAlicaEngineInfo(const AlicaEngineInfo& bi) const
{
    alica_msgs::msg::AlicaEngineInfo bis;
    bis.current_plan = bi.currentPlan;
    bis.current_role = bi.currentRole;
    bis.current_state = bi.currentState;
    bis.current_task = bi.currentTask;
    bis.master_plan = bi.masterPlan;

    for (const auto& robotID : bi.robotIDsWithMe) {
        alica_msgs::msg::AllocationAuthorityInfo::_sender_id_type rosRobotID;
        rosRobotID = robotID;
        bis.robot_ids_with_me.push_back(rosRobotID);
    }

    bis.sender_id = bi.senderID;

    if (_isRunning) {
        _alicaEngineInfoPublisher->publish(bis);
    }
}

void AlicaRosCommunicationCommon::sendPlanTreeInfo(const PlanTreeInfo& pti) const
{
    alica_msgs::msg::PlanTreeInfo ptis;
    ptis.sender_id = pti.senderID;

    ptis.state_ids.reserve(pti.stateIDs.size());
    for (int64_t i : pti.stateIDs) {
        ptis.state_ids.push_back(i);
    }
    ptis.succeeded_eps.reserve(pti.succeededEPs.size());
    for (int64_t i : pti.succeededEPs) {
        ptis.succeeded_eps.push_back(i);
    }
    if (_isRunning) {
        _planTreeInfoPublisher->publish(ptis);
    }
}

void AlicaRosCommunicationCommon::sendRoleSwitch(const RoleSwitch& rs, AgentId agentID) const
{
    alica_msgs::msg::RoleSwitch rss;

    rss.role_id = rs.roleID;
    rss.sender_id = agentID;

    if (_isRunning) {
        _roleSwitchPublisher->publish(rss);
    }
}

void AlicaRosCommunicationCommon::sendSyncReady(const SyncReady& sr) const
{
    alica_msgs::msg::SyncReady srs;

    srs.sender_id = sr.senderID;
    srs.sync_transition_id = sr.synchronisationID;

    if (_isRunning) {
        _syncReadyPublisher->publish(srs);
    }
}

void AlicaRosCommunicationCommon::sendSyncTalk(const SyncTalk& st) const
{
    alica_msgs::msg::SyncTalk sts;
    sts.sender_id = st.senderID;

    for (auto sd : st.syncData) {
        alica_msgs::msg::SyncData sds;
        sds.ack = sd.ack;
        sds.condition_holds = sd.conditionHolds;
        sds.robot_id = sd.agentID;
        sds.transition_id = sd.transitionID;
        sts.sync_data.push_back(sds);
    }

    if (_isRunning) {
        _syncTalkPublisher->publish(sts);
    }
}

void AlicaRosCommunicationCommon::sendSolverResult(const SolverResult& sr) const
{
    alica_msgs::msg::SolverResult srs;
    srs.sender_id = sr.senderID;

    for (const SolverVar& sv : sr.vars) {
        alica_msgs::msg::SolverVar svs;
        svs.id = sv.id;
        svs.value = sv.value;
        srs.vars.push_back(std::move(svs));
    }

    if (_isRunning) {
        _solverResultPublisher->publish(srs);
    }
}

void AlicaRosCommunicationCommon::handleAllocationAuthorityRos(const alica_msgs::msg::AllocationAuthorityInfo& aaimsg)
{
    AllocationAuthorityInfo aai;
    aai.senderID = aaimsg.sender_id;
    aai.planId = aaimsg.plan_id;
    aai.parentState = aaimsg.parent_state;
    aai.planType = aaimsg.plan_type;
    aai.authority = aaimsg.authority;

    aai.entryPointRobots.reserve(aaimsg.entrypoints.size());
    for (const auto& ep : aaimsg.entrypoints) {
        alica::EntryPointRobots newEP;
        newEP.entrypoint = ep.entry_point;
        newEP.robots.reserve(ep.robots.size());
        for (auto robotID : ep.robots) {
            newEP.robots.push_back(robotID);
        }

        aai.entryPointRobots.push_back(newEP);
    }

    if (_isRunning) {
        onAuthorityInfoReceived(aai);
    }
}

void AlicaRosCommunicationCommon::handlePlanTreeInfoRos(const alica_msgs::msg::PlanTreeInfo& pti)
{
    auto ptiPtr = make_shared<PlanTreeInfo>();
    ptiPtr->senderID = pti.sender_id;
    for (int64_t i : pti.state_ids) {
        ptiPtr->stateIDs.push_back(i);
    }
    for (int64_t i : pti.succeeded_eps) {
        ptiPtr->succeededEPs.push_back(i);
    }

    if (_isRunning) {
        onPlanTreeInfoReceived(ptiPtr);
    }
}

void AlicaRosCommunicationCommon::handleSyncReadyRos(const alica_msgs::msg::SyncReady& sr)
{
    auto srPtr = make_shared<SyncReady>();
    srPtr->senderID = sr.sender_id;
    srPtr->synchronisationID = sr.sync_transition_id;
    if (_isRunning) {
        onSyncReadyReceived(srPtr);
    }
}

void AlicaRosCommunicationCommon::handleSyncTalkRos(const alica_msgs::msg::SyncTalk& st)
{
    auto stPtr = make_shared<SyncTalk>();
    stPtr->senderID = st.sender_id;
    stPtr->syncData.reserve(st.sync_data.size());
    for (const auto& sd : st.sync_data) {
        SyncData sds = SyncData();
        sds.ack = sd.ack;
        sds.conditionHolds = sd.condition_holds;
        sds.agentID = sd.robot_id;
        sds.transitionID = sd.transition_id;
        stPtr->syncData.push_back(std::move(sds));
    }

    if (_isRunning) {
        onSyncTalkReceived(stPtr);
    }
}

void AlicaRosCommunicationCommon::handleSolverResult(const alica_msgs::msg::SolverResult& sr)
{
    SolverResult osr;
    osr.senderID = sr.sender_id;
    osr.vars.reserve(sr.vars.size());

    for (const auto& sv : sr.vars) {
        SolverVar svs;
        svs.id = sv.id;
        svs.value = sv.value;
        osr.vars.push_back(std::move(svs));
    }

    if (_isRunning) {
        onSolverResult(osr);
    }
}

void AlicaRosCommunicationCommon::handleAgentQuery(const alica_msgs::msg::AgentQuery& pq)
{
    AgentQuery newpq;
    newpq.senderID = pq.sender_id;
    newpq.senderSdk = pq.sender_sdk;
    newpq.planHash = pq.plan_hash;

    if (_isRunning) {
        onAgentQuery(newpq);
    }
}

void AlicaRosCommunicationCommon::handleAgentAnnouncement(const alica_msgs::msg::AgentAnnouncement& pa)
{
    AgentAnnouncement newpa;
    newpa.senderID = pa.sender_id;
    newpa.token = pa.token;
    newpa.senderName = pa.sender_name;
    newpa.senderSdk = pa.sender_sdk;
    newpa.planHash = pa.plan_hash;
    newpa.roleId = pa.role_id;
    for (const alica_msgs::msg::StringTuple& st : pa.capabilities) {
        newpa.capabilities.push_back(std::make_pair(st.key, st.value));
    }

    if (_isRunning) {
        onAgentAnnouncement(newpa);
    }
}

void AlicaRosCommunicationCommon::sendAgentQuery(const AgentQuery& pq) const
{
    alica_msgs::msg::AgentQuery newpq;
    newpq.sender_id = pq.senderID;
    newpq.sender_sdk = pq.senderSdk;
    newpq.plan_hash = pq.planHash;
    if (_isRunning) {
        _presenceQueryPublisher->publish(newpq);
    }
}

void AlicaRosCommunicationCommon::sendAgentAnnouncement(const AgentAnnouncement& pa) const
{
    alica_msgs::msg::AgentAnnouncement newpa;
    newpa.sender_id = pa.senderID;
    newpa.token = pa.token;
    newpa.sender_name = pa.senderName;
    newpa.sender_sdk = pa.senderSdk;
    newpa.plan_hash = pa.planHash;
    newpa.role_id = pa.roleId;
    for (const auto& cap : pa.capabilities) {
        alica_msgs::msg::StringTuple st;
        st.key = cap.first;
        st.value = cap.second;
        newpa.capabilities.push_back(std::move(st));
    }

    if (_isRunning) {
        _presenceAnnouncementPublisher->publish(newpa);
    }
}

void AlicaRosCommunicationCommon::sendLogMessage(int level, const string& message) const
{
    switch (level) {
    case int(rclcpp::Logger::Level::Debug):
        RCLCPP_DEBUG(_node->get_logger(), "AlicaMessage: %s", message.c_str());
        break;
    case int(rclcpp::Logger::Level::Info):
        RCLCPP_INFO(_node->get_logger(), "AlicaMessage: %s", message.c_str());
        break;
    case int(rclcpp::Logger::Level::Warn):
        RCLCPP_WARN(_node->get_logger(), "AlicaMessage: %s", message.c_str());
        break;
    case int(rclcpp::Logger::Level::Error):
        RCLCPP_ERROR(_node->get_logger(), "AlicaMessage: %s", message.c_str());
        break;
    case int(rclcpp::Logger::Level::Fatal):
        RCLCPP_FATAL(_node->get_logger(), "AlicaMessage: %s", message.c_str());
        break;
    default:
        RCLCPP_ERROR(_node->get_logger(), "AlicaMessage: %s", message.c_str());
        break;
    }
}

void AlicaRosCommunicationCommon::startCommunication()
{
    _isRunning = true;
}
void AlicaRosCommunicationCommon::stopCommunication()
{
    _isRunning = false;
    // Waits for subscription callbacks that are in progress: the engine that they call can be destroyed after this returns.
    // Callbacks that start later see that the communication is not running.
    std::unique_lock<std::shared_mutex> lock(_callbackGuard->mutex);
}

} /* namespace alicaRosProxy */
