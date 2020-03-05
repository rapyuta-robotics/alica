#include "communication/AlicaRosCommunication.h"

#include <essentials/Configuration.h>
#include <engine/AlicaEngine.h>
#include <engine/containers/AlicaEngineInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/containers/RoleSwitch.h>
#include <engine/containers/SolverResult.h>
#include <engine/containers/SolverVar.h>
#include <engine/containers/SyncData.h>
#include <engine/containers/SyncReady.h>
#include <engine/containers/SyncTalk.h>
#include <engine/containers/AgentQuery.h>
#include <engine/teammanager/TeamManager.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

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
}

AlicaRosCommunication::AlicaRosCommunication(AlicaEngine* ae)
        : IAlicaCommunication(ae)
{
    _isRunning = false;
    _rosNode = new ros::NodeHandle();
    _spinner = new ros::AsyncSpinner(4);

    _allocationAuthorityInfoPublisher = _rosNode->advertise<alica_msgs::AllocationAuthorityInfo>(allocationAuthorityInfoTopic, 2);
    _allocationAuthorityInfoSubscriber =
            _rosNode->subscribe(allocationAuthorityInfoTopic, 10, &AlicaRosCommunication::handleAllocationAuthorityRos, (AlicaRosCommunication*) this);

    _alicaEngineInfoPublisher = _rosNode->advertise<alica_msgs::AlicaEngineInfo>(alicaEngineInfoTopic, 2);
    _roleSwitchPublisher = _rosNode->advertise<alica_msgs::RoleSwitch>(ownRoleTopic, 10);

    _planTreeInfoPublisher = _rosNode->advertise<alica_msgs::PlanTreeInfo>(planTreeInfoTopic, 10);
    _planTreeInfoSubscriber = _rosNode->subscribe(planTreeInfoTopic, 5, &AlicaRosCommunication::handlePlanTreeInfoRos, (AlicaRosCommunication*) this);

    _syncReadyPublisher = _rosNode->advertise<alica_msgs::SyncReady>(syncReadyTopic, 10);
    _syncReadySubscriber = _rosNode->subscribe(syncReadyTopic, 5, &AlicaRosCommunication::handleSyncReadyRos, (AlicaRosCommunication*) this);

    _syncTalkPublisher = _rosNode->advertise<alica_msgs::SyncTalk>(syncTalkTopic, 10);
    _syncTalkSubscriber = _rosNode->subscribe(syncTalkTopic, 5, &AlicaRosCommunication::handleSyncTalkRos, (AlicaRosCommunication*) this);

    _solverResultPublisher = _rosNode->advertise<alica_msgs::SolverResult>(solverResultTopic, 10);
    _solverResultSubscriber = _rosNode->subscribe(solverResultTopic, 5, &AlicaRosCommunication::handleSolverResult, (AlicaRosCommunication*) this);

    _presenceQueryPublisher = _rosNode->advertise<alica_msgs::AgentQuery>(presenceQueryTopic, 5, true);
    _presenceQuerySubscriber = _rosNode->subscribe(presenceQueryTopic, 5, &AlicaRosCommunication::handleAgentQuery, (AlicaRosCommunication*) this);

    _presenceAnnouncementPublisher = _rosNode->advertise<alica_msgs::AgentAnnouncement>(presenceAnnouncementTopic, 5, true);
    _presenceAnnouncementSubscriber =
            _rosNode->subscribe(presenceAnnouncementTopic, 50, &AlicaRosCommunication::handleAgentAnnouncement, (AlicaRosCommunication*) this);
}

AlicaRosCommunication::~AlicaRosCommunication()
{
    if (_isRunning) {
        _spinner->stop();
    }
    delete _spinner;

    _allocationAuthorityInfoSubscriber.shutdown();
    _roleSwitchPublisher.shutdown();
    _planTreeInfoSubscriber.shutdown();
    _syncReadySubscriber.shutdown();
    _syncTalkSubscriber.shutdown();
    _rosNode->shutdown();
    delete _rosNode;
}

void AlicaRosCommunication::tick()
{
    if (_isRunning) {
        // Use this for synchronous communication!
        // ros::spinOnce();
    }
}

void AlicaRosCommunication::sendAllocationAuthority(const AllocationAuthorityInfo& aai) const
{
    alica_msgs::AllocationAuthorityInfo aais;

    aais.plan_id = aai.planId;
    aais.parent_state = aai.parentState;
    aais.plan_type = aai.planType;

    aais.sender_id.id = aai.senderID->toByteVector();
    aais.authority.id = aai.authority->toByteVector();

    for (auto& ep : aai.entryPointRobots) {
        alica_msgs::EntryPointRobots newEP;
        newEP.entry_point = ep.entrypoint;
        int i = 0;
        for (auto& agentId : ep.robots) {
            newEP.robots.push_back(alica_msgs::EntryPointRobots::_robots_type::value_type());
            for (size_t j = 0; j < agentId->getSize(); j++) {
                newEP.robots[i].id.push_back(*(agentId->getRaw() + j));
            }
            i++;
        }
        aais.entrypoints.push_back(newEP);
    }

    if (_isRunning) {
        _allocationAuthorityInfoPublisher.publish(aais);
    }
}

void AlicaRosCommunication::sendAlicaEngineInfo(const AlicaEngineInfo& bi) const
{
    alica_msgs::AlicaEngineInfo bis;
    bis.current_plan = bi.currentPlan;
    bis.current_role = bi.currentRole;
    bis.current_state = bi.currentState;
    bis.current_task = bi.currentTask;
    bis.master_plan = bi.masterPlan;

    for (auto& robotID : bi.robotIDsWithMe) {
        alica_msgs::AllocationAuthorityInfo::_sender_id_type rosRobotID;
        rosRobotID.id = robotID->toByteVector();
        bis.robot_ids_with_me.push_back(rosRobotID);
    }

    bis.sender_id.id = bi.senderID->toByteVector();

    if (_isRunning) {
        _alicaEngineInfoPublisher.publish(bis);
    }
}

void AlicaRosCommunication::sendPlanTreeInfo(const PlanTreeInfo& pti) const
{
    alica_msgs::PlanTreeInfo ptis;
    ptis.sender_id.id = pti.senderID->toByteVector();

    ptis.state_ids.reserve(pti.stateIDs.size());
    for (int64_t i : pti.stateIDs) {
        ptis.state_ids.push_back(i);
    }
    ptis.succeeded_eps.reserve(pti.succeededEPs.size());
    for (int64_t i : pti.succeededEPs) {
        ptis.succeeded_eps.push_back(i);
    }
    if (_isRunning) {
        _planTreeInfoPublisher.publish(ptis);
    }
}

void AlicaRosCommunication::sendRoleSwitch(const RoleSwitch& rs) const
{
    alica_msgs::RoleSwitch rss;

    rss.role_id = rs.roleID;
    auto robotID = ae->getTeamManager().getLocalAgentID();
    rss.sender_id.id = robotID->toByteVector();

    if (_isRunning) {
        _roleSwitchPublisher.publish(rss);
    }
}

void AlicaRosCommunication::sendSyncReady(const SyncReady& sr) const
{
    alica_msgs::SyncReady srs;

    srs.sender_id.id = sr.senderID->toByteVector();
    srs.sync_transition_id = sr.synchronisationID;

    if (_isRunning) {
        _syncReadyPublisher.publish(srs);
    }
}

void AlicaRosCommunication::sendSyncTalk(const SyncTalk& st) const
{
    alica_msgs::SyncTalk sts;
    sts.sender_id.id = st.senderID->toByteVector();

    for (auto sd : st.syncData) {
        alica_msgs::SyncData sds;
        sds.ack = sd.ack;
        sds.condition_holds = sd.conditionHolds;
        sds.robot_id.id = sd.robotID->toByteVector();
        sds.transition_id = sd.transitionID;
        sts.sync_data.push_back(sds);
    }

    if (_isRunning) {
        _syncTalkPublisher.publish(sts);
    }
}

void AlicaRosCommunication::sendSolverResult(const SolverResult& sr) const
{
    alica_msgs::SolverResult srs;
    srs.sender_id.id = sr.senderID->toByteVector();

    for (const SolverVar& sv : sr.vars) {
        alica_msgs::SolverVar svs;
        svs.id = sv.id;
        svs.value = std::vector<uint8_t>(sv.value, sv.value + sizeof(sv.value) / sizeof(sv.value[0]));
        srs.vars.push_back(std::move(svs));
    }

    if (_isRunning) {
        _solverResultPublisher.publish(srs);
    }
}

void AlicaRosCommunication::handleAllocationAuthorityRos(const alica_msgs::AllocationAuthorityInfo& aaimsg)
{
    AllocationAuthorityInfo aai;
    aai.senderID = this->ae->getIDFromBytes(aaimsg.sender_id.id.data(), aaimsg.sender_id.id.size(), aaimsg.sender_id.type);
    aai.planId = aaimsg.plan_id;
    aai.parentState = aaimsg.parent_state;
    aai.planType = aaimsg.plan_type;
    aai.authority = this->ae->getIDFromBytes(aaimsg.authority.id.data(), aaimsg.authority.id.size(), aaimsg.authority.type);

    aai.entryPointRobots.reserve(aaimsg.entrypoints.size());
    for (const auto& ep : aaimsg.entrypoints) {
        alica::EntryPointRobots newEP;
        newEP.entrypoint = ep.entry_point;
        newEP.robots.reserve(ep.robots.size());
        for (auto robotID : ep.robots) {
            newEP.robots.push_back(this->ae->getIDFromBytes(robotID.id.data(), robotID.id.size(), robotID.type));
        }

        aai.entryPointRobots.push_back(newEP);
    }

    if (_isRunning) {
        onAuthorityInfoReceived(aai);
    }
}

void AlicaRosCommunication::handlePlanTreeInfoRos(alica_msgs::PlanTreeInfoPtr pti)
{
    auto ptiPtr = make_shared<PlanTreeInfo>();
    ptiPtr->senderID = this->ae->getIDFromBytes(pti->sender_id.id.data(), pti->sender_id.id.size(), pti->sender_id.type);
    for (int64_t i : pti->state_ids) {
        ptiPtr->stateIDs.push_back(i);
    }
    for (int64_t i : pti->succeeded_eps) {
        ptiPtr->succeededEPs.push_back(i);
    }

    if (_isRunning) {
        onPlanTreeInfoReceived(ptiPtr);
    }
}

void AlicaRosCommunication::handleSyncReadyRos(alica_msgs::SyncReadyPtr sr)
{
    auto srPtr = make_shared<SyncReady>();
    srPtr->senderID = this->ae->getIDFromBytes(sr->sender_id.id.data(), sr->sender_id.id.size(), sr->sender_id.type);
    srPtr->synchronisationID = sr->sync_transition_id;
    if (_isRunning) {
        onSyncReadyReceived(srPtr);
    }
}

void AlicaRosCommunication::handleSyncTalkRos(alica_msgs::SyncTalkPtr st)
{
    auto stPtr = make_shared<SyncTalk>();
    stPtr->senderID = this->ae->getIDFromBytes(st->sender_id.id.data(), st->sender_id.id.size(), st->sender_id.type);
    stPtr->syncData.reserve(st->sync_data.size());
    for (const auto& sd : st->sync_data) {
        SyncData sds = SyncData();
        sds.ack = sd.ack;
        sds.conditionHolds = sd.condition_holds;
        sds.robotID = this->ae->getIDFromBytes(sd.robot_id.id.data(),sd.robot_id.id.size(), sd.robot_id.type);
        sds.transitionID = sd.transition_id;
        stPtr->syncData.push_back(std::move(sds));
    }

    if (_isRunning) {
        onSyncTalkReceived(stPtr);
    }
}

void AlicaRosCommunication::handleSolverResult(const alica_msgs::SolverResult& sr)
{
    SolverResult osr;
    osr.senderID = this->ae->getIDFromBytes(sr.sender_id.id.data(), sr.sender_id.id.size(), sr.sender_id.type);
    osr.vars.reserve(sr.vars.size());

    for (const auto& sv : sr.vars) {
        SolverVar svs;
        svs.id = sv.id;
        std::copy(sv.value.begin(), sv.value.end(), svs.value);
        osr.vars.push_back(std::move(svs));
    }

    if (_isRunning) {
        onSolverResult(osr);
    }
}

void AlicaRosCommunication::handleAgentQuery(const alica_msgs::AgentQuery& pq)
{
    AgentQuery newpq;
    newpq.senderID = this->ae->getIDFromBytes(pq.sender_id.id.data(), pq.sender_id.id.size(), pq.sender_id.type);
    newpq.senderSdk = pq.sender_sdk;
    newpq.planHash = pq.plan_hash;

    if (_isRunning) {
        onAgentQuery(newpq);
    }
}

void AlicaRosCommunication::handleAgentAnnouncement(const alica_msgs::AgentAnnouncement& pa)
{
    AgentAnnouncement newpa;
    newpa.senderID = this->ae->getIDFromBytes(pa.sender_id.id.data(), pa.sender_id.id.size(), pa.sender_id.type);
    newpa.token = pa.token;
    newpa.senderName = pa.sender_name;
    newpa.senderSdk = pa.sender_sdk;
    newpa.planHash = pa.plan_hash;
    newpa.roleId = pa.roleId;
    for (const alica_msgs::StringTuple& st : pa.capabilities) {
        newpa.capabilities.push_back(std::make_pair(st.key, st.value));
    }

    if (_isRunning) {
        onAgentAnnouncement(newpa);
    }
}

void AlicaRosCommunication::sendAgentQuery(const AgentQuery& pq) const
{
    alica_msgs::AgentQuery newpq;
    newpq.sender_id.id = pq.senderID->toByteVector();
    newpq.sender_sdk = pq.senderSdk;
    newpq.plan_hash = pq.planHash;
    if (_isRunning) {
        _presenceQueryPublisher.publish(newpq);
    }
}

void AlicaRosCommunication::sendAgentAnnouncement(const AgentAnnouncement& pa) const
{
    alica_msgs::AgentAnnouncement newpa;
    newpa.sender_id.id = pa.senderID->toByteVector();
    newpa.token = pa.token;
    newpa.sender_name = pa.senderName;
    newpa.sender_sdk = pa.senderSdk;
    newpa.plan_hash = pa.planHash;
    newpa.roleId = pa.roleId;
    for (const auto& cap : pa.capabilities) {
        alica_msgs::StringTuple st;
        st.key = cap.first;
        st.value = cap.second;
        newpa.capabilities.push_back(std::move(st));
    }

    if (_isRunning) {
        _presenceAnnouncementPublisher.publish(newpa);
    }
}

void AlicaRosCommunication::sendLogMessage(int level, const string& message) const
{
    switch (level) {
    case ::ros::console::levels::Debug:
        ROS_DEBUG("AlicaMessage: %s", message.c_str());
        break;
    case ::ros::console::levels::Info:
        ROS_INFO("AlicaMessage: %s", message.c_str());
        break;
    case ::ros::console::levels::Warn:
        ROS_WARN("AlicaMessage: %s", message.c_str());
        break;
    case ::ros::console::levels::Error:
        ROS_ERROR("AlicaMessage: %s", message.c_str());
        break;
    case ::ros::console::levels::Fatal:
        ROS_FATAL("AlicaMessage: %s", message.c_str());
        break;
    default:
        ROS_ERROR("AlicaMessage: %s", message.c_str());
        break;
    }
}

void AlicaRosCommunication::startCommunication()
{
    _isRunning = true;
    _spinner->start();
}
void AlicaRosCommunication::stopCommunication()
{
    _isRunning = false;
    _spinner->stop();
}

} /* namespace alicaRosProxy */
