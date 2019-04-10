#include "communication/AlicaCapnzeroCommunication.h"

// Alica Containers:
#include "engine/containers/AlicaEngineInfo.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/containers/SolverResult.h"
#include "engine/containers/SolverVar.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"

//#include "msgs/AlicaEngineInfo.capnp.h"

#include <capnzero/CapnZero.h>
#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>

using namespace alica;

namespace alicaCapnzeroProxy {
    using std::make_shared;
    using std::string;

    AlicaCapnzeroCommunication::AlicaCapnzeroCommunication(AlicaEngine *ae) : IAlicaCommunication(ae) {
        this->isRunning = false;

        // Create zmq context
        this->ctx = zmq_ctx_new();
        this->url = "224.0.0.2:5555";

        // Find topics:
        // The values are hardcoded for initial testing. Will be replaced by proper config management later!
        this->allocationAuthorityInfoTopic = "allocAutInfo";
        this->ownRoleTopic = "myRole";
        this->alicaEngineInfoTopic = "EngineInfo";
        this->planTreeInfoTopic = "planTree";
        this->syncReadyTopic = "syncRDY";
        this->syncTalkTopic = "syncTLK";
        this->solverResultTopic = "solverRES";

        // Setup publishers:
        this->AlicaEngineInfoPublisher = new capnzero::Publisher(this->ctx, this->alicaEngineInfoTopic);
        this->RoleSwitchPublisher = new capnzero::Publisher(this->ctx, this->ownRoleTopic);
        this->AllocationAuthorityInfoPublisher = new capnzero::Publisher(this->ctx, this->allocationAuthorityInfoTopic);
        this->PlanTreeInfoPublisher = new capnzero::Publisher(this->ctx, this->planTreeInfoTopic);
        this->SyncReadyPublisher = new capnzero::Publisher(this->ctx, this->syncReadyTopic);
        this->SyncTalkPublisher = new capnzero::Publisher(this->ctx, this->syncTalkTopic);
        this->SolverResultPublisher = new capnzero::Publisher(this->ctx, this->solverResultTopic);

        // Open sockets:
        this->AlicaEngineInfoPublisher->bind(capnzero::CommType::UDP, this->url);
        this->RoleSwitchPublisher->bind(capnzero::CommType::UDP, this->url);
        this->AllocationAuthorityInfoPublisher->bind(capnzero::CommType::UDP, this->url);


        // Setup Subscribers:
        this->AlicaEngineInfoSubscriber = new capnzero::Subscriber(this->ctx, this->alicaEngineInfoTopic);
        this->RoleSwitchSubscriber = new capnzero::Subscriber(this->ctx, this->ownRoleTopic);
        this->AllocationAuthorityInfoSubscriber = new capnzero::Subscriber(this->ctx,
                                                                           this->allocationAuthorityInfoTopic);
        this->PlanTreeInfoSubscriber = new capnzero::Subscriber(this->ctx, this->planTreeInfoTopic);
        this->SyncReadySubscriber = new capnzero::Subscriber(this->ctx, this->syncReadyTopic);
        this->SyncTalkSubscriber = new capnzero::Subscriber(this->ctx, this->syncTalkTopic);
        this->SolverResultSubscriber = new capnzero::Subscriber(this->ctx, this->solverResultTopic);
    }

    AlicaCapnzeroCommunication::~AlicaCapnzeroCommunication() {
        // Delete Publishers:
        delete this->SolverResultPublisher;
        delete this->SyncTalkPublisher;
        delete this->SyncReadyPublisher;
        delete this->PlanTreeInfoPublisher;
        delete this->AllocationAuthorityInfoPublisher;
        delete this->RoleSwitchPublisher;
        delete this->AlicaEngineInfoPublisher;

        // Delete Subscribers:
        delete this->SolverResultSubscriber;
        delete this->SyncTalkSubscriber;
        delete this->SyncReadySubscriber;
        delete this->PlanTreeInfoSubscriber;
        delete this->AllocationAuthorityInfoSubscriber;
        delete this->RoleSwitchSubscriber;
        delete this->AlicaEngineInfoSubscriber;

        // Delete zmq context:
        zmq_ctx_term(this->ctx);
    }

    void AlicaCapnzeroCommunication::sendAllocationAuthority(const AllocationAuthorityInfo &aai) const {
//        alica_msgs::AllocationAuthorityInfo aais;
//
//        aais.plan_id = aai.planId;
//        aais.parent_state = aai.parentState;
//        aais.plan_type = aai.planType;
//
//        aais.sender_id.id = aai.senderID->toByteVector();
//        aais.authority.id = aai.authority->toByteVector();

//        for (auto& ep : aai.entryPointRobots) {
//            alica_msgs::EntryPointRobots newEP;
//            newEP.entry_point = ep.entrypoint;
//            int i = 0;
//            for (auto& robotId : ep.robots) {
//                newEP.robots.push_back(alica_msgs::EntryPointRobots::_robots_type::value_type());
//                for (int j = 0; j < robotId->getSize(); j++) {
//                    newEP.robots[i].id.push_back(*(robotId->getRaw() + j));
//                }
//                i++;
//            }
//            aais.entrypoints.push_back(newEP);
//        }

        if (this->isRunning) {
//            unsigned int bytes_sent = this->AllocationAuthorityInfoPublisher->send();
        }
    }

    void AlicaCapnzeroCommunication::sendAlicaEngineInfo(const AlicaEngineInfo &bi) const {
        //    alica_msgs::AlicaEngineInfo bis;
        //    bis.current_plan = bi.currentPlan;
        //    bis.current_role = bi.currentRole;
        //    bis.current_state = bi.currentState;
        //    bis.current_task = bi.currentTask;
        //    bis.master_plan = bi.masterPlan;
        //
        //    for (auto& robotID : bi.robotIDsWithMe) {
        //        alica_msgs::AllocationAuthorityInfo::_sender_id_type rosRobotID;
        //        rosRobotID.id = robotID->toByteVector();
        //        bis.robot_ids_with_me.push_back(rosRobotID);
        //    }
        //
        //    bis.sender_id.id = bi.senderID->toByteVector();
        //
        //    if (this->isRunning) {
        //        this->AlicaEngineInfoPublisher.publish(bis);
        //    }
    }

    void AlicaCapnzeroCommunication::sendPlanTreeInfo(const PlanTreeInfo &pti) const {
        //    alica_msgs::PlanTreeInfo ptis;
        //    ptis.sender_id.id = pti.senderID->toByteVector();
        //
        //    ptis.state_ids.reserve(pti.stateIDs.size());
        //    for (int64_t i : pti.stateIDs) {
        //        ptis.state_ids.push_back(i);
        //    }
        //    ptis.succeeded_eps.reserve(pti.succeededEPs.size());
        //    for (int64_t i : pti.succeededEPs) {
        //        ptis.succeeded_eps.push_back(i);
        //    }
        //    if (this->isRunning) {
        //        this->PlanTreeInfoPublisher.publish(ptis);
        //    }
    }

    void AlicaCapnzeroCommunication::sendRoleSwitch(const RoleSwitch &rs) const {
        //    alica_msgs::RoleSwitch rss;
        //
        //    rss.role_id = rs.roleID;
        //    auto robotID = this->ae->getTeamManager()->getLocalAgentID();
        //    rss.sender_id.id = robotID->toByteVector();
        //
        //    if (this->isRunning) {
        //        this->RoleSwitchPublisher.publish(rss);
        //    }
    }

    void AlicaCapnzeroCommunication::sendSyncReady(const SyncReady &sr) const {
        //    alica_msgs::SyncReady srs;
        //
        //    srs.sender_id.id = sr.senderID->toByteVector();
        //    srs.synchronisation_id = sr.synchronisationID;
        //
        //    if (this->isRunning) {
        //        this->SyncReadyPublisher.publish(srs);
        //    }
    }

    void AlicaCapnzeroCommunication::sendSyncTalk(const SyncTalk &st) const {
        //    alica_msgs::SyncTalk sts;
        //    sts.sender_id.id = st.senderID->toByteVector();
        //
        //    for (auto sd : st.syncData) {
        //        alica_msgs::SyncData sds;
        //        sds.ack = sd.ack;
        //        sds.condition_holds = sd.conditionHolds;
        //        sds.robot_id.id = sd.robotID->toByteVector();
        //        sds.transition_id = sd.transitionID;
        //        sts.sync_data.push_back(sds);
        //    }
        //
        //    if (this->isRunning) {
        //        this->SyncTalkPublisher.publish(sts);
        //    }
    }

    void AlicaCapnzeroCommunication::sendSolverResult(const SolverResult &sr) const {
        //    alica_msgs::SolverResult srs;
        //    srs.sender_id.id = sr.senderID->toByteVector();
        //
        //    for (const SolverVar& sv : sr.vars) {
        //        alica_msgs::SolverVar svs;
        //        svs.id = sv.id;
        //        svs.value = std::vector<uint8_t>(sv.value, sv.value + sizeof(sv.value) / sizeof(sv.value[0]));
        //        srs.vars.push_back(std::move(svs));
        //    }
        //
        //    if (this->isRunning) {
        //        this->SolverResultPublisher.publish(srs);
        //    }
    }

    void AlicaCapnzeroCommunication::handleAllocationAuthority(const alica_msgs::AllocationAuthorityInfo &aaimsg) {
        //    AllocationAuthorityInfo aai;
        //    aai.senderID = this->ae->getIdFromBytes(aaimsg.sender_id.id);
        //    aai.planId = aaimsg.plan_id;
        //    aai.parentState = aaimsg.parent_state;
        //    aai.planType = aaimsg.plan_type;
        //    aai.authority = this->ae->getIdFromBytes(aaimsg.authority.id);
        //
        //    aai.entryPointRobots.reserve(aaimsg.entrypoints.size());
        //    for (const auto& ep : aaimsg.entrypoints) {
        //        alica::EntryPointRobots newEP;
        //        newEP.entrypoint = ep.entry_point;
        //        newEP.robots.reserve(ep.robots.size());
        //        for (auto robotID : ep.robots) {
        //            newEP.robots.push_back(this->ae->getIdFromBytes(robotID.id));
        //        }
        //
        //        aai.entryPointRobots.push_back(newEP);
        //    }
        //
        //    if (this->isRunning) {
        //        onAuthorityInfoReceived(aai);
        //    }
    }

    void AlicaCapnzeroCommunication::handlePlanTreeInfo(alica_msgs::PlanTreeInfoPtr pti) {
        //    auto ptiPtr = make_shared<PlanTreeInfo>();
        //    ptiPtr->senderID = this->ae->getIdFromBytes(pti->sender_id.id);
        //    for (int64_t i : pti->state_ids) {
        //        ptiPtr->stateIDs.push_back(i);
        //    }
        //    for (int64_t i : pti->succeeded_eps) {
        //        ptiPtr->succeededEPs.push_back(i);
        //    }
        //
        //    if (this->isRunning) {
        //        this->onPlanTreeInfoReceived(ptiPtr);
        //    }
    }

    void AlicaCapnzeroCommunication::handleSyncReady(alica_msgs::SyncReadyPtr sr) {
        //    auto srPtr = make_shared<SyncReady>();
        //    srPtr->senderID = this->ae->getIdFromBytes(sr->sender_id.id);
        //    srPtr->synchronisationID = sr->synchronisation_id;
        //
        //    if (this->isRunning) {
        //        this->onSyncReadyReceived(srPtr);
        //    }
    }

    void AlicaCapnzeroCommunication::handleSyncTalk(alica_msgs::SyncTalkPtr st) {
        //    auto stPtr = make_shared<SyncTalk>();
        //    stPtr->senderID = this->ae->getIdFromBytes(st->sender_id.id);
        //
        //    stPtr->syncData.reserve(st->sync_data.size());
        //    for (const auto& sd : st->sync_data) {
        //        SyncData sds = SyncData();
        //        sds.ack = sd.ack;
        //        sds.conditionHolds = sd.condition_holds;
        //        sds.robotID = this->ae->getIdFromBytes(sd.robot_id.id);
        //        sds.transitionID = sd.transition_id;
        //        stPtr->syncData.push_back(std::move(sds));
        //    }
        //
        //    if (this->isRunning) {
        //        this->onSyncTalkReceived(stPtr);
        //    }
    }

    void AlicaCapnzeroCommunication::handleSolverResult(const alica_msgs::SolverResult &sr) {
        //    SolverResult osr;
        //    osr.senderID = this->ae->getIdFromBytes(sr.sender_id.id);
        //    osr.vars.reserve(sr.vars.size());
        //
        //    for (const auto& sv : sr.vars) {
        //        SolverVar svs;
        //        svs.id = sv.id;
        //        std::copy(sv.value.begin(), sv.value.end(), svs.value);
        //        osr.vars.push_back(std::move(svs));
        //    }
        //
        //    if (isRunning) {
        //        onSolverResult(osr);
        //    }
    }

    void AlicaCapnzeroCommunication::sendLogMessage(int level, const string &message) const {
        //    switch (level) {
        //        case ::ros::console::levels::Debug:
        //            ROS_DEBUG("AlicaMessage: %s", message.c_str());
        //            break;
        //        case ::ros::console::levels::Info:
        //            ROS_INFO("AlicaMessage: %s", message.c_str());
        //            break;
        //        case ::ros::console::levels::Warn:
        //            ROS_WARN("AlicaMessage: %s", message.c_str());
        //            break;
        //        case ::ros::console::levels::Error:
        //            ROS_ERROR("AlicaMessage: %s", message.c_str());
        //            break;
        //        case ::ros::console::levels::Fatal:
        //            ROS_FATAL("AlicaMessage: %s", message.c_str());
        //            break;
        //        default:
        //            ROS_ERROR("AlicaMessage: %s", message.c_str());
        //            break;
        //    }
    }

    void AlicaCapnzeroCommunication::startCommunication() {
        //    this->isRunning = true;
        //    spinner->start();
    }

    void AlicaCapnzeroCommunication::stopCommunication() {
        //    this->isRunning = false;
        //    spinner->stop();
    }
}