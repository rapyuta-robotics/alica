#include "communication/AlicaRosCommunication.h"

#include "Configuration.h"
#include "SystemConfig.h"
#include "engine/AlicaEngine.h"
#include "engine/containers/AlicaEngineInfo.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/containers/SolverResult.h"
#include "engine/containers/SolverVar.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/teammanager/TeamManager.h"
#include "supplementary/AgentIDFactory.h"

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

using namespace alica;

namespace alicaRosProxy
{

using std::make_shared;
using std::string;

AlicaRosCommunication::AlicaRosCommunication(AlicaEngine* ae)
        : IAlicaCommunication(ae)
{
    this->isRunning = false;
    rosNode = new ros::NodeHandle();
    spinner = new ros::AsyncSpinner(4);

    // read topic strings from AlicaRosProxy.conf
    this->sc = supplementary::SystemConfig::getInstance();
    this->allocationAuthorityInfoTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.allocationAuthorityInfoTopic", NULL);
    this->ownRoleTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.ownRoleTopic", NULL);
    this->alicaEngineInfoTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.alicaEngineInfoTopic", NULL);
    this->planTreeInfoTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.planTreeInfoTopic", NULL);
    this->syncReadyTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.syncReadyTopic", NULL);
    this->syncTalkTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.syncTalkTopic", NULL);
    this->solverResultTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.solverResultTopic", NULL);

    AllocationAuthorityInfoPublisher = rosNode->advertise<alica_msgs::AllocationAuthorityInfo>(this->allocationAuthorityInfoTopic, 2);
    AllocationAuthorityInfoSubscriber =
            rosNode->subscribe(this->allocationAuthorityInfoTopic, 10, &AlicaRosCommunication::handleAllocationAuthorityRos, (AlicaRosCommunication*) this);

    AlicaEngineInfoPublisher = rosNode->advertise<alica_msgs::AlicaEngineInfo>(this->alicaEngineInfoTopic, 2);
    RoleSwitchPublisher = rosNode->advertise<alica_msgs::RoleSwitch>(this->ownRoleTopic, 10);

    PlanTreeInfoPublisher = rosNode->advertise<alica_msgs::PlanTreeInfo>(this->planTreeInfoTopic, 10);
    PlanTreeInfoSubscriber = rosNode->subscribe(this->planTreeInfoTopic, 1, &AlicaRosCommunication::handlePlanTreeInfoRos, (AlicaRosCommunication*) this);

    SyncReadyPublisher = rosNode->advertise<alica_msgs::SyncReady>(this->syncReadyTopic, 10);
    SyncReadySubscriber = rosNode->subscribe(this->syncReadyTopic, 5, &AlicaRosCommunication::handleSyncReadyRos, (AlicaRosCommunication*) this);
    SyncTalkPublisher = rosNode->advertise<alica_msgs::SyncTalk>(this->syncTalkTopic, 10);
    SyncTalkSubscriber = rosNode->subscribe(this->syncTalkTopic, 5, &AlicaRosCommunication::handleSyncTalkRos, (AlicaRosCommunication*) this);

    SolverResultPublisher = rosNode->advertise<alica_msgs::SolverResult>(this->solverResultTopic, 10);
    SolverResultSubscriber = rosNode->subscribe(this->solverResultTopic, 5, &AlicaRosCommunication::handleSolverResult, (AlicaRosCommunication*) this);
}

AlicaRosCommunication::~AlicaRosCommunication()
{
    if (this->isRunning) {
        spinner->stop();
    }
    delete spinner;

    AllocationAuthorityInfoSubscriber.shutdown();
    RoleSwitchPublisher.shutdown();
    PlanTreeInfoSubscriber.shutdown();
    SyncReadySubscriber.shutdown();
    SyncTalkSubscriber.shutdown();
    rosNode->shutdown();
    delete rosNode;
}

void AlicaRosCommunication::tick()
{
    if (this->isRunning) {
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
        for (auto& robotId : ep.robots) {
            newEP.robots.push_back(alica_msgs::EntryPointRobots::_robots_type::value_type());
            for (int j = 0; j < robotId->getSize(); j++) {
                newEP.robots[i].id.push_back(*(robotId->getRaw() + j));
            }
            i++;
        }
        aais.entrypoints.push_back(newEP);
    }

    if (this->isRunning) {
        this->AllocationAuthorityInfoPublisher.publish(aais);
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

    for (const auto& robotID : bi.robotIDsWithMe) {
        alica_msgs::AllocationAuthorityInfo::_sender_id_type rosRobotID;
        rosRobotID.id = robotID->toByteVector();
        bis.robot_ids_with_me.push_back(std::move(rosRobotID));
    }

    bis.sender_id.id = bi.senderID->toByteVector();

    if (this->isRunning) {
        this->AlicaEngineInfoPublisher.publish(bis);
    }
}

void AlicaRosCommunication::sendPlanTreeInfo(const PlanTreeInfo& pti) const
{
    alica_msgs::PlanTreeInfo ptis;
    ptis.sender_id.id = pti.senderID->toByteVector();

    for (int64_t i : pti.stateIDs) {
        ptis.state_ids.push_back(i);
    }
    for (int64_t i : pti.succeededEPs) {
        ptis.succeeded_eps.push_back(i);
    }
    if (this->isRunning) {
        this->PlanTreeInfoPublisher.publish(ptis);
    }
}

void AlicaRosCommunication::sendRoleSwitch(const RoleSwitch& rs) const
{
    alica_msgs::RoleSwitch rss;

    rss.role_id = rs.roleID;
    auto robotID = this->ae->getTeamManager()->getLocalAgentID();
    rss.sender_id.id = robotID->toByteVector();

    if (this->isRunning) {
        this->RoleSwitchPublisher.publish(rss);
    }
}

void AlicaRosCommunication::sendSyncReady(const SyncReady& sr) const
{
    alica_msgs::SyncReady srs;

    srs.sender_id.id = sr.senderID->toByteVector();
    srs.sync_transition_id = sr.syncTransitionID;

    if (this->isRunning) {
        this->SyncReadyPublisher.publish(srs);
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

    if (this->isRunning) {
        this->SyncTalkPublisher.publish(sts);
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

    if (this->isRunning) {
        this->SolverResultPublisher.publish(srs);
    }
}

void AlicaRosCommunication::handleAllocationAuthorityRos(const alica_msgs::AllocationAuthorityInfo& aaimsg)
{
    AllocationAuthorityInfo aai;
    aai.senderID = this->ae->getIdFromBytes(aaimsg.sender_id.id);
    aai.planId = aaimsg.plan_id;
    aai.parentState = aaimsg.parent_state;
    aai.planType = aaimsg.plan_type;
    aai.authority = this->ae->getIdFromBytes(aaimsg.authority.id);

    aai.entryPointRobots.reserve(aaimsg.entrypoints.size());
    for (const auto& ep : aaimsg.entrypoints) {
        alica::EntryPointRobots newEP;
        newEP.entrypoint = ep.entry_point;
        newEP.robots.reserve(ep.robots.size());
        for (const auto& robotID : ep.robots) {
            newEP.robots.push_back(this->ae->getIdFromBytes(robotID.id));
        }

        aai.entryPointRobots.push_back(newEP);
    }

    if (this->isRunning) {
        onAuthorityInfoReceived(aai);
    }
}

void AlicaRosCommunication::handlePlanTreeInfoRos(alica_msgs::PlanTreeInfoPtr pti)
{
    auto ptiPtr = make_shared<PlanTreeInfo>();
    ptiPtr->senderID = this->ae->getIdFromBytes(pti->sender_id.id);
    for (int64_t i : pti->state_ids) {
        ptiPtr->stateIDs.push_back(i);
    }
    for (int64_t i : pti->succeeded_eps) {
        ptiPtr->succeededEPs.push_back(i);
    }

    if (this->isRunning) {
        this->onPlanTreeInfoReceived(ptiPtr);
    }
}

void AlicaRosCommunication::handleSyncReadyRos(alica_msgs::SyncReadyPtr sr)
{
    auto srPtr = make_shared<SyncReady>();
    srPtr->senderID = this->ae->getIdFromBytes(sr->sender_id.id);
    srPtr->syncTransitionID = sr->sync_transition_id;

    if (this->isRunning) {
        this->onSyncReadyReceived(srPtr);
    }
}

void AlicaRosCommunication::handleSyncTalkRos(alica_msgs::SyncTalkPtr st)
{
    auto stPtr = make_shared<SyncTalk>();
    stPtr->senderID = this->ae->getIdFromBytes(st->sender_id.id);

    for (const auto& sd : st->sync_data) {
        SyncData sds = SyncData();
        sds.ack = sd.ack;
        sds.conditionHolds = sd.condition_holds;
        sds.robotID = this->ae->getIdFromBytes(sd.robot_id.id);
        sds.transitionID = sd.transition_id;
        stPtr->syncData.push_back(std::move(sds));
    }

    if (this->isRunning) {
        this->onSyncTalkReceived(stPtr);
    }
}

void AlicaRosCommunication::handleSolverResult(const alica_msgs::SolverResult& sr)
{
    SolverResult osr;
    osr.senderID = this->ae->getIdFromBytes(sr.sender_id.id);
    osr.vars.reserve(sr.vars.size());

    for (const auto& sv : sr.vars) {
        SolverVar svs;
        svs.id = sv.id;
        std::copy(sv.value.begin(), sv.value.end(), svs.value);
        osr.vars.push_back(std::move(svs));
    }

    if (isRunning) {
        onSolverResult(osr);
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
    this->isRunning = true;
    spinner->start();
}
void AlicaRosCommunication::stopCommunication()
{
    this->isRunning = false;
    spinner->stop();
}

} /* namespace alicaRosProxy */
