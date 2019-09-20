#include "alica_capnzero_proxy/Communication.h"

#include "alica_capnzero_proxy/ContainerUtils.h"

// Generated CapnProto Messages:
#include "alica_msg/AlicaEngineInfo.capnp.h"
#include "alica_msg/AllocationAuthorityInfo.capnp.h"
#include "alica_msg/PlanTreeInfo.capnp.h"
#include "alica_msg/RoleSwitch.capnp.h"
#include "alica_msg/SolverResult.capnp.h"
#include "alica_msg/SyncReady.capnp.h"
#include "alica_msg/SyncTalk.capnp.h"

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

namespace alica_capnzero_proxy
{
using std::make_shared;
using std::string;

Communication::Communication(alica::AlicaEngine* ae)
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
    this->alicaPub = new capnzero::Publisher(this->ctx, capnzero::Protocol::UDP);
    this->alicaPub->setDefaultTopic("ALICA");

    // Open sockets:
    this->alicaPub->addAddress(this->url);

    // Setup Subscribers:
    this->allocationAuthorityInfoSub = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->allocationAuthorityInfoSub->setTopic(this->allocationAuthorityInfoTopic);
    this->planTreeInfoSub = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->planTreeInfoSub->setTopic(this->planTreeInfoTopic);
    this->syncReadySub = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->syncReadySub->setTopic(this->syncReadyTopic);
    this->syncTalkSub = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->syncTalkSub->setTopic(this->syncTalkTopic);
    this->solverResultSub = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->solverResultSub->setTopic(this->solverResultTopic);

    // connecting the subscribers:
    this->allocationAuthorityInfoSub->addAddress(this->url);
    this->planTreeInfoSub->addAddress(this->url);
    this->syncReadySub->addAddress(this->url);
    this->syncTalkSub->addAddress(this->url);
    this->solverResultSub->addAddress(this->url);

    // subscribing the subscribers:
    this->allocationAuthorityInfoSub->subscribe(&Communication::handleAllocationAuthority, &(*this));
    this->planTreeInfoSub->subscribe(&Communication::handlePlanTreeInfo, &(*this));
    this->syncReadySub->subscribe(&Communication::handleSyncReady, &(*this));
    this->syncTalkSub->subscribe(&Communication::handleSyncTalk, &(*this));
    this->solverResultSub->subscribe(&Communication::handleSolverResult, &(*this));
}

Communication::~Communication()
{
    // Delete Publishers:
    delete this->alicaPub;

    // Delete Subscribers:
    delete this->solverResultSub;
    delete this->syncTalkSub;
    delete this->syncReadySub;
    delete this->planTreeInfoSub;
    delete this->allocationAuthorityInfoSub;

    // Delete zmq context:
    zmq_ctx_shutdown(this->ctx);
    zmq_ctx_term(this->ctx);
}

void Communication::sendAllocationAuthority(const alica::AllocationAuthorityInfo& aai) const
{
    if (!this->isRunning) {
        return;
    }
    ::capnp::MallocMessageBuilder msgBuilder;
    ContainerUtils::toMsg(aai, msgBuilder);
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "Communication: Sending AAI: " << msgBuilder.getRoot<alica_msgs::AllocationAuthorityInfo>().toString().flatten().cStr() << '\n';
#endif
    this->alicaPub->send(msgBuilder, this->allocationAuthorityInfoTopic);
}

void Communication::sendAlicaEngineInfo(const alica::AlicaEngineInfo& aei) const
{
    if (!this->isRunning) {
        return;
    }
    ::capnp::MallocMessageBuilder msgBuilder;
    ContainerUtils::toMsg(aei, msgBuilder);
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "Communication: Sending AEI: " << msgBuilder.getRoot<alica_msgs::AlicaEngineInfo>().toString().flatten().cStr() << '\n';
#endif
    this->alicaPub->send(msgBuilder, this->alicaEngineInfoTopic);
}

void Communication::sendPlanTreeInfo(const alica::PlanTreeInfo& pti) const
{
    if (!this->isRunning) {
        return;
    }
    ::capnp::MallocMessageBuilder msgBuilder;
    ContainerUtils::toMsg(pti, msgBuilder);
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "Communication: Sending PTI: " << msgBuilder.getRoot<alica_msgs::PlanTreeInfo>().toString().flatten().cStr() << '\n';
#endif
    this->alicaPub->send(msgBuilder, this->planTreeInfoTopic);
}

void Communication::sendRoleSwitch(const alica::RoleSwitch& rs) const
{
    if (!this->isRunning) {
        return;
    }
    ::capnp::MallocMessageBuilder msgBuilder;
    ContainerUtils::toMsg(rs, msgBuilder);
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "Communication: Sending RS: " << msgBuilder.getRoot<alica_msgs::RoleSwitch>().toString().flatten().cStr() << '\n';
#endif
    this->alicaPub->send(msgBuilder, this->ownRoleTopic);
}

void Communication::sendSyncReady(const alica::SyncReady& sr) const
{
    if (!this->isRunning) {
        return;
    }
    ::capnp::MallocMessageBuilder msgBuilder;
    ContainerUtils::toMsg(sr, msgBuilder);
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "Communication: Sending SR: " << msgBuilder.getRoot<alica_msgs::SyncReady>().toString().flatten().cStr() << '\n';
#endif
    this->alicaPub->send(msgBuilder, this->syncReadyTopic);
}

void Communication::sendSyncTalk(const alica::SyncTalk& st) const
{
    if (!this->isRunning) {
        return;
    }
    ::capnp::MallocMessageBuilder msgBuilder;
    ContainerUtils::toMsg(st, msgBuilder);
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "Communication: Sending ST: " << msgBuilder.getRoot<alica_msgs::SyncReady>().toString().flatten().cStr() << '\n';
#endif
    this->alicaPub->send(msgBuilder, this->syncTalkTopic);
}

void Communication::sendSolverResult(const alica::SolverResult& sr) const {
    if (!this->isRunning) {
        return;
    }
    ::capnp::MallocMessageBuilder msgBuilder;
    ContainerUtils::toMsg(sr, msgBuilder);
#ifdef CAPNZERO_PROXY_DEBUG
    std::cout << "Communication: Sending SR: " << msgBuilder.getRoot<alica_msgs::SolverResult>().toString().flatten().cStr() << '\n';
#endif
    this->alicaPub->send(msgBuilder, this->solverResultTopic);
}

void Communication::handleAllocationAuthority(::capnp::FlatArrayMessageReader& msg)
{
    if (!this->isRunning) {
        return;
    }
    onAuthorityInfoReceived(ContainerUtils::toAllocationAuthorityInfo(msg, this->ae->getIdManager()));
}

void Communication::handlePlanTreeInfo(::capnp::FlatArrayMessageReader& msg)
{
    if (!this->isRunning) {
        return;
    }
    // NOTE: Didn't know how to create shared_ptr from a stack object, but why the hell is it a shared_ptr that the engine needs???
    alica::PlanTreeInfo pti = ContainerUtils::toPlanTreeInfo(msg, this->ae->getIdManager());
    std::shared_ptr<alica::PlanTreeInfo> ptiPtr = std::make_shared<alica::PlanTreeInfo>();
    ptiPtr->senderID = pti.senderID;
    for (auto stateId : pti.stateIDs) {
        ptiPtr->stateIDs.push_back(stateId);
    }
    for (auto epId : pti.succeededEPs) {
        ptiPtr->succeededEPs.push_back(epId);
    }
    this->onPlanTreeInfoReceived(ptiPtr);
}

void Communication::handleSyncReady(::capnp::FlatArrayMessageReader& msg)
{
    if (!this->isRunning) {
        return;
    }
    auto srPtr = make_shared<alica::SyncReady>();
    alica::SyncReady sr = ContainerUtils::toSyncReady(msg, this->ae->getIdManager());
    srPtr->senderID = sr.senderID;
    srPtr->synchronisationID = sr.synchronisationID;
    this->onSyncReadyReceived(srPtr);
}

void Communication::handleSyncTalk(::capnp::FlatArrayMessageReader& msg)
{
    if (!this->isRunning) {
        return;
    }
    auto stPtr = make_shared<alica::SyncTalk>();
    alica::SyncTalk st = ContainerUtils::toSyncTalk(msg, this->ae->getIdManager());
    stPtr->senderID = st.senderID;
    for (auto sd : st.syncData) {
        stPtr->syncData.push_back(sd);
    }
    this->onSyncTalkReceived(stPtr);
}

void Communication::handleSolverResult(::capnp::FlatArrayMessageReader& msg)
{
    if (!this->isRunning) {
        return;
    }
    onSolverResult(ContainerUtils::toSolverResult(msg, this->ae->getIdManager()));
}

void Communication::sendLogMessage(int level, const string& message) const
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

void Communication::startCommunication()
{
    this->isRunning = true;
}

void Communication::stopCommunication()
{
    this->isRunning = false;
}
} // namespace alica_capnzero_proxy
