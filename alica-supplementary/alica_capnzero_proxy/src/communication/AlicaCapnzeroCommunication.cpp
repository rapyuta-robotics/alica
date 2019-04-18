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

// Messages:
#include "msgs/AllocationAuthorityInfo.capnp.h"
#include "msgs/AlicaEngineInfo.capnp.h"
#include "msgs/PlanTreeInfo.capnp.h"
#include "msgs/RoleSwitch.capnp.h"
#include "msgs/SolverResult.capnp.h"
#include "msgs/SyncReady.capnp.h"
#include "msgs/SyncTalk.capnp.h"

#include <capnzero/CapnZero.h>
#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>
#include <iostream>

#include <essentials/AgentID.h>
#include <essentials/AgentIDFactory.h>

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
        std::cout << "The publisher:\n";
        this->AlicaPublisher = new capnzero::Publisher(this->ctx);
        this->AlicaPublisher->setDefaultGroup("ALICA");

        // Open sockets:
        this->AlicaPublisher->bind(capnzero::CommType::UDP, this->url);


        // Setup Subscribers:
        std::cout << "The subscribers:\n";
        this->AlicaEngineInfoSubscriber = new capnzero::Subscriber(this->ctx, this->alicaEngineInfoTopic);
        this->RoleSwitchSubscriber = new capnzero::Subscriber(this->ctx, this->ownRoleTopic);
        this->AllocationAuthorityInfoSubscriber = new capnzero::Subscriber(this->ctx,
                                                                           this->allocationAuthorityInfoTopic);
        this->PlanTreeInfoSubscriber = new capnzero::Subscriber(this->ctx, this->planTreeInfoTopic);
        this->SyncReadySubscriber = new capnzero::Subscriber(this->ctx, this->syncReadyTopic);
        this->SyncTalkSubscriber = new capnzero::Subscriber(this->ctx, this->syncTalkTopic);
        this->SolverResultSubscriber = new capnzero::Subscriber(this->ctx, this->solverResultTopic);

        // connecting the subscribers:
        this->AllocationAuthorityInfoSubscriber->connect(capnzero::CommType::UDP, this->url);

        // subscribing the subscribers:
        this->AllocationAuthorityInfoSubscriber->subscribe(&AlicaCapnzeroCommunication::handleAllocationAuthority, &(*this));
    }

    AlicaCapnzeroCommunication::~AlicaCapnzeroCommunication() {
        // Delete Publishers:
        delete this->AlicaPublisher;

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
        ::capnp::MallocMessageBuilder msgBuilder;
        alica_capnp_msgs::AllocationAuthorityInfo::Builder msg = msgBuilder.initRoot<alica_capnp_msgs::AllocationAuthorityInfo>();

        std::cout << "Sending: " << aai;

        msg.setParentState(aai.parentState);
        msg.setPlanId(aai.planId);
        msg.setParentState(aai.parentState);
        msg.setPlanType(aai.planType);
        UUID::Builder sender = msg.initSenderId();
        sender.setValue(kj::arrayPtr(aai.senderID->getRaw(), (unsigned int) aai.senderID->getSize()));
        UUID::Builder authority = msg.initAuthority();
        authority.setValue(kj::arrayPtr(aai.authority->getRaw(),  (unsigned int) aai.authority->getSize()));
        ::capnp::List<alica_capnp_msgs::EntrypointRobots>::Builder entrypoints = msg.initEntrypoints((unsigned int) aai.entryPointRobots.size());
        for (unsigned int i = 0; i < aai.entryPointRobots.size(); ++i) {
            auto ep = aai.entryPointRobots[i];
            alica_capnp_msgs::EntrypointRobots::Builder tmp = entrypoints[i];
            tmp.setEntrypoint(ep.entrypoint);
            ::capnp::List<UUID>::Builder tmpRobots = tmp.initRobots((unsigned int) ep.robots.size());
            for (int j = 0; j < ep.robots.size(); ++i) {
                 UUID::Builder tmpUUID = tmpRobots[j];
                 tmpUUID.setValue(kj::arrayPtr(ep.robots[j]->getRaw(), (unsigned int) ep.robots[j]->getSize()));
            }
        }

        if (this->isRunning) {
            std::cout << "Sending data: " << msg.toString().flatten().cStr() << '\n';
            this->AlicaPublisher->send(msgBuilder, this->allocationAuthorityInfoTopic);
        }
    }

    void AlicaCapnzeroCommunication::sendAlicaEngineInfo(const alica::AlicaEngineInfo &bi) const {
        ::capnp::MallocMessageBuilder msgBuilder;
        alica_capnp_msgs::AlicaEngineInfo::Builder msg = msgBuilder.initRoot<alica_capnp_msgs::AlicaEngineInfo>();

        UUID::Builder sender = msg.initSenderId();
        sender.setValue(kj::arrayPtr(bi.senderID->getRaw(), (unsigned int) bi.senderID->getSize()));

        msg.setMasterPlan(bi.masterPlan);
        msg.setCurrentPlan(bi.currentPlan);
        msg.setCurrentRole(bi.currentRole);
        msg.setCurrentState(bi.currentState);
        msg.setCurrentTask(bi.currentTask);

        ::capnp::List<UUID>::Builder agents = msg.initAgentIdsWithMe((unsigned int) bi.robotIDsWithMe.size());
        for (int i = 0; i < bi.robotIDsWithMe.size(); ++i) {
            auto &robo = bi.robotIDsWithMe[i];
            UUID::Builder tmp = agents[0];
            tmp.setValue(kj::arrayPtr(robo->getRaw(), (unsigned int) robo->getSize()));
        }

        if (this->isRunning) {
            this->AlicaPublisher->send(msgBuilder, this->alicaEngineInfoTopic);
        }
    }

    void AlicaCapnzeroCommunication::sendPlanTreeInfo(const alica::PlanTreeInfo &pti) const {
        ::capnp::MallocMessageBuilder msgBuilder;
        alica_capnp_msgs::PlanTreeInfo::Builder msg = msgBuilder.initRoot<alica_capnp_msgs::PlanTreeInfo>();
        UUID::Builder sender = msg.initSenderId();
        sender.setValue(kj::arrayPtr(pti.senderID->getRaw(), (unsigned int) pti.senderID->getSize()));
        ::capnp::List<int64_t>::Builder stateIds = msg.initStateIds((unsigned int) pti.stateIDs.size());
        for (unsigned int i = 0; i < pti.stateIDs.size(); ++i) {
            stateIds.set(i, pti.stateIDs[i]);
        }
        ::capnp::List<int64_t>::Builder succededEps = msg.initSucceededEps((unsigned int) pti.succeededEPs.size());
        for (unsigned int i = 0; i < pti.succeededEPs.size(); ++i) {
            succededEps.set(i, pti.succeededEPs[i]);
        }

            if (this->isRunning) {
                this->AlicaPublisher->send(msgBuilder, this->planTreeInfoTopic);
            }
    }

    void AlicaCapnzeroCommunication::sendRoleSwitch(const alica::RoleSwitch &rs) const {
        // TODO: Uncomment if testing is not needed anymore
        ::capnp::MallocMessageBuilder msgBuilder;
        alica_capnp_msgs::RoleSwitch::Builder msg = msgBuilder.initRoot<alica_capnp_msgs::RoleSwitch>();
//        std::vector<uint8_t> robotID = this->ae->getTeamManager()->getLocalAgentID();
        UUID::Builder sender = msg.initSenderId();
//        sender.setValue(kj::arrayPtr(robotID.data(), robotID.size() * sizeof(uint8_t)));
        msg.setRoleId(rs.roleID);

        if (this->isRunning) {
            this->AlicaPublisher->send(msgBuilder, this->ownRoleTopic);
        }
    }

    void AlicaCapnzeroCommunication::sendSyncReady(const alica::SyncReady &sr) const {
        ::capnp::MallocMessageBuilder msgBuilder;
        alica_capnp_msgs::SyncReady::Builder msg = msgBuilder.initRoot<alica_capnp_msgs::SyncReady>();

        UUID::Builder sender = msg.initSenderId();
        sender.setValue(kj::arrayPtr(sr.senderID->getRaw(), (unsigned int) sr.senderID->getSize()));
        msg.setSynchronisationId(sr.synchronisationID);

        if (this->isRunning) {
            this->AlicaPublisher->send(msgBuilder, this->syncReadyTopic);
        }
    }

    void AlicaCapnzeroCommunication::sendSyncTalk(const alica::SyncTalk &st) const {
        ::capnp::MallocMessageBuilder msgBuilder;
        alica_capnp_msgs::SyncTalk::Builder msg = msgBuilder.initRoot<alica_capnp_msgs::SyncTalk>();
        UUID::Builder sender = msg.initSenderId();
        sender.setValue(kj::arrayPtr(st.senderID->getRaw(), (unsigned int) st.senderID->getSize()));

        ::capnp::List<alica_capnp_msgs::SyncData>::Builder syncData= msg.initSyncData((unsigned int) st.syncData.size());
        for (int i = 0; i < st.syncData.size(); ++i) {
            auto &ds = st.syncData[i];
            alica_capnp_msgs::SyncData::Builder tmpData = syncData[i];
            UUID::Builder tmpId = tmpData.initRobotId();
            tmpId.setValue(kj::arrayPtr(ds.robotID->getRaw(), (unsigned int) ds.robotID->getSize()));
            tmpData.setAck(ds.ack);
            tmpData.setTransitionHolds(ds.conditionHolds);
            tmpData.setTransitionId(ds.transitionID);
        }

        if (this->isRunning) {
            this->AlicaPublisher->send(msgBuilder, this->syncTalkTopic);
        }
    }

    void AlicaCapnzeroCommunication::sendSolverResult(const alica::SolverResult &sr) const {
        ::capnp::MallocMessageBuilder msgBuilder;
        alica_capnp_msgs::SolverResult::Builder msg = msgBuilder.initRoot<alica_capnp_msgs::SolverResult>();
        UUID::Builder sender = msg.initSenderId();
        sender.setValue(kj::arrayPtr(sr.senderID->getRaw(), (unsigned int) sr.senderID->getSize()));
        ::capnp::List<alica_capnp_msgs::SolverVar>::Builder vars = msg.initVars((unsigned int) sr.vars.size());
        for (int i = 0; i < sr.vars.size(); ++i) {
            auto &var = sr.vars[i];
            alica_capnp_msgs::SolverVar::Builder tmpVar = vars[i];
            tmpVar.setId(var.id);
            tmpVar.setValue(kj::arrayPtr(var.value, sizeof(var.value)));
        }

        if (this->isRunning) {
            this->AlicaPublisher->send(msgBuilder, this->solverResultTopic);
        }
    }

    void AlicaCapnzeroCommunication::handleAllocationAuthority(::capnp::FlatArrayMessageReader& msg) {
        alica::AllocationAuthorityInfo aai;
        std::cout << "handleAllocationAuthority called.\n";
        std::cout << "Recieved data: " << msg.getRoot<alica_capnp_msgs::AllocationAuthorityInfo>().toString().flatten().cStr() << std::endl;
        alica_capnp_msgs::AllocationAuthorityInfo::Reader reader = msg.getRoot<alica_capnp_msgs::AllocationAuthorityInfo>();
        std::vector<uint8_t> id;
        id.assign(reader.getSenderId().getValue().begin(), reader.getSenderId().getValue().end());
        aai.senderID = essentials::AgentIDFactory().create(id);
        aai.planId = reader.getPlanId();
        aai.planType = reader.getPlanType();
        aai.parentState = reader.getParentState();
        id.clear();
        id.assign(reader.getAuthority().getValue().begin(), reader.getAuthority().getValue().end());
        aai.authority = essentials::AgentIDFactory().create(id);
        ::capnp::List<alica_capnp_msgs::EntrypointRobots>::Reader entrypoints = reader.getEntrypoints();
        std::vector<alica::EntryPointRobots> epData;
        for (int i = 0; i < entrypoints.size(); ++i) {
            EntryPointRobots tmp;
            alica_capnp_msgs::EntrypointRobots::Reader tmpEp = entrypoints[i];
            tmp.entrypoint = tmpEp.getEntrypoint();
            std::vector<essentials::AgentID> agentIds;
            ::capnp::List<UUID>::Reader robots = tmpEp.getRobots();
            for (int j = 0; j < robots.size(); ++j) { // Not shure if this works!!!
                id.clear();
                UUID::Reader tmpUUID = robots[j];
                id.assign(tmpUUID.getValue().begin(), tmpUUID.getValue().end());
                alica::AgentIDConstPtr tmpItem = essentials::AgentIDFactory().create(id);
                agentIds.push_back(*tmpItem);
            }
        }

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
            if (this->isRunning) {
//                onAuthorityInfoReceived(aai);
            }
    }

    void AlicaCapnzeroCommunication::handlePlanTreeInfo(::capnp::FlatArrayMessageReader& msg) {
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

    void AlicaCapnzeroCommunication::handleSyncReady(::capnp::FlatArrayMessageReader& msg) {
        //    auto srPtr = make_shared<SyncReady>();
        //    srPtr->senderID = this->ae->getIdFromBytes(sr->sender_id.id);
        //    srPtr->synchronisationID = sr->synchronisation_id;
        //
        //    if (this->isRunning) {
        //        this->onSyncReadyReceived(srPtr);
        //    }
    }

    void AlicaCapnzeroCommunication::handleSyncTalk(::capnp::FlatArrayMessageReader& msg) {
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

    void AlicaCapnzeroCommunication::handleSolverResult(::capnp::FlatArrayMessageReader& msg) {
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
        switch (level) {
            case 1:
                std::cout << "AlicaMessage[DBG]: " << message << '\n'; // DEBUG
                break;
            case 2:
                std::cout << "AlicaMessage[INF]: \033[97m" << message << "\033[0m\n"; // INFO
                break;
            case 3:
                std::cout << "AlicaMessage[WRN]: \033[34m" << message << "\033[0m\n"; // WARNING
                break;
            case 4:
                std::cerr << "\033[31mAlicaMessage[ERR]: " << message << "\033[0m\n"; // ERROR
                break;
            case 5:
                std::cerr << "\033[91mAlicaMessage[CRT]: " << message << "\033[0m\n"; // CRITICAL
                break;
            default:
                std::cerr << "\033[31mAlicaMessage[ERR]: " << message << "\033[0m\n"; // default to ERROR
                break;
        }
    }

    void AlicaCapnzeroCommunication::startCommunication() {
            this->isRunning = true;
    }

    void AlicaCapnzeroCommunication::stopCommunication() {
            this->isRunning = false;
    }
}