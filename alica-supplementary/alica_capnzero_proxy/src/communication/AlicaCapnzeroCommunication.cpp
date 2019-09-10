#include "communication/AlicaCapnzeroCommunication.h"

// Generated CapnProto Messages:
#include "alica_capnz_msg/AlicaEngineInfo.capnp.h"
#include "alica_capnz_msg/AllocationAuthorityInfo.capnp.h"
#include "alica_capnz_msg/PlanTreeInfo.capnp.h"
#include "alica_capnz_msg/RoleSwitch.capnp.h"
#include "alica_capnz_msg/SolverResult.capnp.h"
#include "alica_capnz_msg/SyncReady.capnp.h"
#include "alica_capnz_msg/SyncTalk.capnp.h"

#include <engine/AlicaEngine.h>
#include <engine/containers/AlicaEngineInfo.h>
#include <engine/containers/AllocationAuthorityInfo.h>
#include <engine/containers/PlanTreeInfo.h>
#include <engine/containers/RoleSwitch.h>
#include <engine/containers/SolverResult.h>
#include <engine/containers/SolverVar.h>
#include <engine/containers/SyncData.h>
#include <engine/containers/SyncReady.h>
#include <engine/containers/SyncTalk.h>
#include <engine/teammanager/TeamManager.h>

#include <SystemConfig.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <capnzero/CapnZero.h>
#include <kj/array.h>

#include <iostream>

//#define CAPNZERO_PROXY_DEBUG

namespace alicaCapnzeroProxy
{
using std::make_shared;
using std::string;

AlicaCapnzeroCommunication::AlicaCapnzeroCommunication(alica::AlicaEngine* ae)
        : IAlicaCommunication(ae)
{
    this->isRunning = false;

    this->sc = essentials::SystemConfig::getInstance();

    // Create zmq context
    this->ctx = zmq_ctx_new();
    this->url = (*sc)["AlicaCapnzProxy"]->get<std::string>("Communication.URL", NULL);
    int tp = (*sc)["AlicaCapnzProxy"]->get<int>("Communication.transport", NULL);
    switch (tp) {
    case 0:
        this->protocol = capnzero::Protocol::UDP;
        break;
    case 1:
        this->protocol = capnzero::Protocol::TCP;
        break;
    case 2:
        this->protocol = capnzero::Protocol::IPC;
        break;
    }

    // Find topics:
    this->allocationAuthorityInfoTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.allocationAuthorityInfoTopic", NULL);
    this->ownRoleTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.ownRoleTopic", NULL);
    this->alicaEngineInfoTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.alicaEngineInfoTopic", NULL);
    this->planTreeInfoTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.planTreeInfoTopic", NULL);
    this->syncReadyTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.syncReadyTopic", NULL);
    this->syncTalkTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.syncTalkTopic", NULL);
    this->solverResultTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.solverResultTopic", NULL);

    // Setup publishers:
    this->AlicaPublisher = new capnzero::Publisher(this->ctx, capnzero::Protocol::UDP);
    this->AlicaPublisher->setDefaultTopic("ALICA");

    // Open sockets:
    this->AlicaPublisher->addAddress(this->url);

    // Setup Subscribers:
    this->AllocationAuthorityInfoSubscriber = new capnzero::Subscriber(this->ctx,capnzero::Protocol::UDP);
    this->AlicaEngineInfoSubscriber->setTopic(this->allocationAuthorityInfoTopic);
    this->PlanTreeInfoSubscriber = new capnzero::Subscriber(this->ctx,capnzero::Protocol::UDP);
    this->PlanTreeInfoSubscriber->setTopic(this->planTreeInfoTopic);
    this->SyncReadySubscriber = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->SyncReadySubscriber->setTopic(this->syncReadyTopic);
    this->SyncTalkSubscriber = new capnzero::Subscriber(this->ctx,capnzero::Protocol::UDP);
    this->SyncTalkSubscriber->setTopic(this->syncTalkTopic);
    this->SolverResultSubscriber = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->SolverResultSubscriber->setTopic(this->solverResultTopic);

    // connecting the subscribers:
    this->AllocationAuthorityInfoSubscriber->addAddress(this->url);
    this->PlanTreeInfoSubscriber->addAddress(this->url);
    this->SyncReadySubscriber->addAddress(this->url);
    this->SyncTalkSubscriber->addAddress(this->url);
    this->SolverResultSubscriber->addAddress(this->url);

    // subscribing the subscribers:
    this->AllocationAuthorityInfoSubscriber->subscribe(&AlicaCapnzeroCommunication::handleAllocationAuthority, &(*this));
    this->PlanTreeInfoSubscriber->subscribe(&AlicaCapnzeroCommunication::handlePlanTreeInfo, &(*this));
    this->SyncReadySubscriber->subscribe(&AlicaCapnzeroCommunication::handleSyncReady, &(*this));
    this->SyncTalkSubscriber->subscribe(&AlicaCapnzeroCommunication::handleSyncTalk, &(*this));
    this->SolverResultSubscriber->subscribe(&AlicaCapnzeroCommunication::handleSolverResult, &(*this));
}

AlicaCapnzeroCommunication::~AlicaCapnzeroCommunication()
{
    // Delete Publishers:
    delete this->AlicaPublisher;

    // Delete Subscribers:
    delete this->SolverResultSubscriber;
    delete this->SyncTalkSubscriber;
    delete this->SyncReadySubscriber;
    delete this->PlanTreeInfoSubscriber;
    delete this->AllocationAuthorityInfoSubscriber;

    // Delete zmq context:
    zmq_ctx_shutdown(this->ctx);
    zmq_ctx_term(this->ctx);
}

void AlicaCapnzeroCommunication::sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const
{
    ::capnp::MallocMessageBuilder msgBuilder;
    alica_capnz_msgs::AllocationAuthorityInfo::Builder msg = msgBuilder.initRoot<alica_capnz_msgs::AllocationAuthorityInfo>();

    msg.setParentState(aai.parentState);
    msg.setPlanId(aai.planId);
    msg.setParentState(aai.parentState);
    msg.setPlanType(aai.planType);
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(aai.senderID->getRaw(), (unsigned int) aai.senderID->getSize()));
    sender.setType(aai.senderID->getType());
    capnzero::ID::Builder authority = msg.initAuthority();
    authority.setValue(kj::arrayPtr(aai.authority->getRaw(), (unsigned int) aai.authority->getSize()));
    ::capnp::List<alica_capnz_msgs::EntrypointRobots>::Builder entrypoints = msg.initEntrypointRobots((unsigned int) aai.entryPointRobots.size());
    for (unsigned int i = 0; i < aai.entryPointRobots.size(); ++i) {
        auto ep = aai.entryPointRobots[i];
        alica_capnz_msgs::EntrypointRobots::Builder tmp = entrypoints[i];
        tmp.setEntrypoint(ep.entrypoint);
        ::capnp::List<capnzero::ID>::Builder tmpRobots = tmp.initRobots((unsigned int) ep.robots.size());
        for (unsigned int j = 0; j < ep.robots.size(); ++i) {
            capnzero::ID::Builder tmpUUID = tmpRobots[j];
            tmpUUID.setValue(kj::arrayPtr(ep.robots[j]->getRaw(), (unsigned int) ep.robots[j]->getSize()));
            tmpUUID.setType(ep.robots[j]->getType());
        }
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: Sending AAI: " << msg.toString().flatten().cStr() << '\n';
#endif
        this->AlicaPublisher->send(msgBuilder, this->allocationAuthorityInfoTopic);
    }
}

void AlicaCapnzeroCommunication::sendAlicaEngineInfo(const alica::AlicaEngineInfo& bi) const
{
    ::capnp::MallocMessageBuilder msgBuilder;
    alica_capnz_msgs::AlicaEngineInfo::Builder msg = msgBuilder.initRoot<alica_capnz_msgs::AlicaEngineInfo>();

    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(bi.senderID->getRaw(), (unsigned int) bi.senderID->getSize()));
    sender.setType(bi.senderID->getType());

    msg.setMasterPlan(bi.masterPlan);
    msg.setCurrentPlan(bi.currentPlan);
    msg.setCurrentRole(bi.currentRole);
    msg.setCurrentState(bi.currentState);
    msg.setCurrentTask(bi.currentTask);

    ::capnp::List<capnzero::ID>::Builder agents = msg.initAgentIdsWithMe((unsigned int) bi.robotIDsWithMe.size());
    for (unsigned int i = 0; i < bi.robotIDsWithMe.size(); ++i) {
        auto& robo = bi.robotIDsWithMe[i];
        capnzero::ID::Builder tmp = agents[0];
        tmp.setValue(kj::arrayPtr(robo->getRaw(), (unsigned int) robo->getSize()));
        tmp.setType(robo->getType());
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: Sending AEI: " << msg.toString().flatten().cStr() << '\n';
#endif
        this->AlicaPublisher->send(msgBuilder, this->alicaEngineInfoTopic);
    }
}

void AlicaCapnzeroCommunication::sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const
{
    ::capnp::MallocMessageBuilder msgBuilder;
    alica_capnz_msgs::PlanTreeInfo::Builder msg = msgBuilder.initRoot<alica_capnz_msgs::PlanTreeInfo>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(pti.senderID->getRaw(), (unsigned int) pti.senderID->getSize()));
    sender.setType(pti.senderID->getType());
    ::capnp::List<int64_t>::Builder stateIds = msg.initStateIds((unsigned int) pti.stateIDs.size());
    for (unsigned int i = 0; i < pti.stateIDs.size(); ++i) {
        stateIds.set(i, pti.stateIDs[i]);
    }
    ::capnp::List<int64_t>::Builder succededEps = msg.initSucceededEps((unsigned int) pti.succeededEPs.size());
    for (unsigned int i = 0; i < pti.succeededEPs.size(); ++i) {
        succededEps.set(i, pti.succeededEPs[i]);
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: Sending PTI: " << msg.toString().flatten().cStr() << '\n';
#endif
        this->AlicaPublisher->send(msgBuilder, this->planTreeInfoTopic);
    }
}

void AlicaCapnzeroCommunication::sendRoleSwitch(const alica::RoleSwitch& rs) const
{
    ::capnp::MallocMessageBuilder msgBuilder;
    alica_capnz_msgs::RoleSwitch::Builder msg = msgBuilder.initRoot<alica_capnz_msgs::RoleSwitch>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(rs.senderID->getRaw(), (unsigned int) rs.senderID->getSize()));
    sender.setType(rs.senderID->getType());
    msg.setRoleId(rs.roleID);

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: Sending RS: " << msg.toString().flatten().cStr() << '\n';
#endif
        this->AlicaPublisher->send(msgBuilder, this->ownRoleTopic);
    }
}

void AlicaCapnzeroCommunication::sendSyncReady(const alica::SyncReady& sr) const
{
    ::capnp::MallocMessageBuilder msgBuilder;
    alica_capnz_msgs::SyncReady::Builder msg = msgBuilder.initRoot<alica_capnz_msgs::SyncReady>();

    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(sr.senderID->getRaw(), (unsigned int) sr.senderID->getSize()));
    sender.setType(sr.senderID->getType());
    msg.setSynchronisationId(sr.synchronisationID);

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: Sending SR: " << msg.toString().flatten().cStr() << '\n';
#endif
        this->AlicaPublisher->send(msgBuilder, this->syncReadyTopic);
    }
}

void AlicaCapnzeroCommunication::sendSyncTalk(const alica::SyncTalk& st) const
{
    ::capnp::MallocMessageBuilder msgBuilder;
    alica_capnz_msgs::SyncTalk::Builder msg = msgBuilder.initRoot<alica_capnz_msgs::SyncTalk>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(st.senderID->getRaw(), (unsigned int) st.senderID->getSize()));
    sender.setType(st.senderID->getType());

    ::capnp::List<alica_capnz_msgs::SyncData>::Builder syncData = msg.initSyncData((unsigned int) st.syncData.size());
    for (unsigned int i = 0; i < st.syncData.size(); ++i) {
        auto& ds = st.syncData[i];
        alica_capnz_msgs::SyncData::Builder tmpData = syncData[i];
        capnzero::ID::Builder tmpId = tmpData.initRobotId();
        tmpId.setValue(kj::arrayPtr(ds.robotID->getRaw(), (unsigned int) ds.robotID->getSize()));
        tmpId.setType(ds.robotID->getType());
        tmpData.setAck(ds.ack);
        tmpData.setTransitionHolds(ds.conditionHolds);
        tmpData.setTransitionId(ds.transitionID);
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: Sending ST: " << msg.toString().flatten().cStr() << '\n';
#endif
        this->AlicaPublisher->send(msgBuilder, this->syncTalkTopic);
    }
}

void AlicaCapnzeroCommunication::sendSolverResult(const alica::SolverResult& sr) const
{
    ::capnp::MallocMessageBuilder msgBuilder;
    alica_capnz_msgs::SolverResult::Builder msg = msgBuilder.initRoot<alica_capnz_msgs::SolverResult>();
    capnzero::ID::Builder sender = msg.initSenderId();
    sender.setValue(kj::arrayPtr(sr.senderID->getRaw(), (unsigned int) sr.senderID->getSize()));
    sender.setType(sr.senderID->getType());
    ::capnp::List<alica_capnz_msgs::SolverVar>::Builder vars = msg.initVars((unsigned int) sr.vars.size());
    for (unsigned int i = 0; i < sr.vars.size(); ++i) {
        auto& var = sr.vars[i];
        alica_capnz_msgs::SolverVar::Builder tmpVar = vars[i];
        tmpVar.setId(var.id);
        tmpVar.setValue(kj::arrayPtr(var.value, sizeof(var.value)));
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: Sending SR: " << msg.toString().flatten().cStr() << '\n';
#endif
        this->AlicaPublisher->send(msgBuilder, this->solverResultTopic);
    }
}

void AlicaCapnzeroCommunication::handleAllocationAuthority(::capnp::FlatArrayMessageReader& msg)
{
    alica::AllocationAuthorityInfo aai;
    alica_capnz_msgs::AllocationAuthorityInfo::Reader reader = msg.getRoot<alica_capnz_msgs::AllocationAuthorityInfo>();
    aai.senderID = this->ae->getIDFromBytes(reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(),
                                            static_cast<uint8_t>(reader.getSenderId().getType()));
    aai.planId = reader.getPlanId();
    aai.planType = reader.getPlanType();
    aai.parentState = reader.getParentState();
    aai.authority = this->ae->getIDFromBytes(reader.getAuthority().getValue().asBytes().begin(), reader.getAuthority().getValue().size(), static_cast<uint8_t>(reader.getAuthority().getType()));
    ::capnp::List<alica_capnz_msgs::EntrypointRobots>::Reader entryPointRobots = reader.getEntrypointRobots();
    for (unsigned int i = 0; i < entryPointRobots.size(); ++i) {
        aai.entryPointRobots.emplace_back();

        alica_capnz_msgs::EntrypointRobots::Reader tmpEntrypointRobot = entryPointRobots[i];
        aai.entryPointRobots[i].entrypoint = tmpEntrypointRobot.getEntrypoint();

        ::capnp::List<capnzero::ID>::Reader robots = tmpEntrypointRobot.getRobots();
        for (unsigned int j = 0; j < robots.size(); ++j) {
            // CapnProto uses uint16_t for enums, but we use uint8_t hopefully it works for us - otherwise we need a matching/translation via switch case
            aai.entryPointRobots[i].robots.push_back(
                    this->ae->getIDFromBytes(robots[j].getValue().asBytes().begin(), robots[j].getValue().size(), (uint8_t) robots[j].getType()));
        }
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: receive: " << reader.toString().flatten().cStr() << std::endl;
#endif
        onAuthorityInfoReceived(aai);
    }
}

void AlicaCapnzeroCommunication::handlePlanTreeInfo(::capnp::FlatArrayMessageReader& msg)
{
    auto ptiPtr = make_shared<alica::PlanTreeInfo>();
    alica_capnz_msgs::PlanTreeInfo::Reader reader = msg.getRoot<alica_capnz_msgs::PlanTreeInfo>();

    ptiPtr->senderID = ae->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), (uint8_t) reader.getSenderId().getType());

    ::capnp::List<int64_t>::Reader states = reader.getStateIds();
    for (unsigned int i = 0; i < states.size(); ++i) {
        ptiPtr->stateIDs.push_back(states[i]);
    }

    ::capnp::List<int64_t>::Reader succeded = reader.getSucceededEps();
    for (unsigned int j = 0; j < succeded.size(); ++j) {
        ptiPtr->succeededEPs.push_back(succeded[j]);
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout <<"AlicaCapnzeroCommunication: receive: " << reader.toString().flatten().cStr() << std::endl;
#endif
        this->onPlanTreeInfoReceived(ptiPtr);
    }
}

void AlicaCapnzeroCommunication::handleSyncReady(::capnp::FlatArrayMessageReader& msg)
{
    auto srPtr = make_shared<alica::SyncReady>();
    alica_capnz_msgs::SyncReady::Reader reader = msg.getRoot<alica_capnz_msgs::SyncReady>();
    srPtr->senderID = ae->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), (uint8_t) reader.getSenderId().getType());
    srPtr->synchronisationID = reader.getSynchronisationId();

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: receive: "  << reader.toString().flatten().cStr() << std::endl;
#endif
        this->onSyncReadyReceived(srPtr);
    }
}

void AlicaCapnzeroCommunication::handleSyncTalk(::capnp::FlatArrayMessageReader& msg)
{
    auto stPtr = make_shared<alica::SyncTalk>();
    alica_capnz_msgs::SyncTalk::Reader reader = msg.getRoot<alica_capnz_msgs::SyncTalk>();
    stPtr->senderID = ae->getIDFromBytes(
            reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(), (uint8_t) reader.getSenderId().getType());
    capnp::List<alica_capnz_msgs::SyncData>::Reader msgSyncData = reader.getSyncData();
    for (unsigned int i = 0; i < msgSyncData.size(); ++i) {
        stPtr->syncData.emplace_back();
        alica_capnz_msgs::SyncData::Reader tmpSyncData = msgSyncData[i];
        stPtr->syncData[i].ack = tmpSyncData.getAck();
        stPtr->syncData[i].conditionHolds = tmpSyncData.getTransitionHolds();
        stPtr->syncData[i].transitionID = tmpSyncData.getTransitionId();
        stPtr->syncData[i].robotID = ae->getIDFromBytes(tmpSyncData.getRobotId().getValue().asBytes().begin(), tmpSyncData.getRobotId().getValue().size(),
                (uint8_t) tmpSyncData.getRobotId().getType());
    }

    if (this->isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: receive: " << reader.toString().flatten().cStr() << std::endl;
#endif
        this->onSyncTalkReceived(stPtr);
    }
}

void AlicaCapnzeroCommunication::handleSolverResult(::capnp::FlatArrayMessageReader& msg)
{
    alica::SolverResult osr;
    alica_capnz_msgs::SolverResult::Reader reader = msg.getRoot<alica_capnz_msgs::SolverResult>();
    osr.senderID = ae->getIDFromBytes(reader.getSenderId().getValue().asBytes().begin(), reader.getSenderId().getValue().size(),
                                   (uint8_t) reader.getSenderId().getType());

    capnp::List<alica_capnz_msgs::SolverVar>::Reader msgSolverVars = reader.getVars();
    for (unsigned int i = 0; i < msgSolverVars.size(); ++i) {
        alica_capnz_msgs::SolverVar::Reader tmpVar = msgSolverVars[i];
        osr.vars.emplace_back();
        osr.vars[i].id = tmpVar.getId();
        std::vector<uint8_t> tmp;
        capnp::List<uint8_t>::Reader val = tmpVar.getValue();
        for (unsigned int j = 0; j < val.size(); ++j) {
            osr.vars[i].value[j] = val[i];
        }
    }

    if (isRunning) {
#ifdef CAPNZERO_PROXY_DEBUG
        std::cout << "AlicaCapnzeroCommunication: receive: " << reader.toString().flatten().cStr() << std::endl;
#endif
        onSolverResult(osr);
    }
}

void AlicaCapnzeroCommunication::sendLogMessage(int level, const string& message) const
{
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

void AlicaCapnzeroCommunication::startCommunication()
{
    this->isRunning = true;
}

void AlicaCapnzeroCommunication::stopCommunication()
{
    this->isRunning = false;
}
} // namespace alicaCapnzeroProxy
