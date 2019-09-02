#include "alica_viewer/alica_viewer_capnzero_interface.h"
#include <alica_capnz_msg/AlicaEngineInfo.capnp.h>
#include <alica_capnz_msg/PlanTreeInfo.capnp.h>

#include <SystemConfig.h>
#include <essentials/NotifyTimer.h>

#include <capnp/common.h>
#include <capnp/message.h>
#include <capnzero/Subscriber.h>
#include <kj/array.h>

namespace alica
{

AlicaViewerCapnzeroInterface::AlicaViewerCapnzeroInterface(int argc, char* argv[])
        : _id_manager(new essentials::IDManager())
{
    // Create zmq context
    this->ctx = zmq_ctx_new();
    this->url = "224.0.0.2:5555";

    this->notifyTimer = new essentials::NotifyTimer<alica::AlicaViewerCapnzeroInterface>(30, &AlicaViewerCapnzeroInterface::timerCallback, this);

    this->sc = essentials::SystemConfig::getInstance();
    this->alicaEngineInfoTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.alicaEngineInfoTopic", NULL);
    this->planTreeInfoTopic = (*sc)["AlicaCapnzProxy"]->get<std::string>("Topics.planTreeInfoTopic", NULL);

    this->AlicaPlanTreeInfoSubscriber = new capnzero::Subscriber(this->ctx, this->planTreeInfoTopic, &AlicaViewerCapnzeroInterface::alicaPlanInfoCallback, &(*this));
    this->AlicaEngineInfoSubscriber = new capnzero::Subscriber(this->ctx, this->alicaEngineInfoTopic, &AlicaViewerCapnzeroInterface::alicaEngineInfoCallback, &(*this));

    this->AlicaPlanTreeInfoSubscriber->addAddress(capnzero::CommType::UDP, this->url);
    this->AlicaEngineInfoSubscriber->addAddress(capnzero::CommType::UDP, this->url);

    this->AlicaPlanTreeInfoSubscriber->connect();
    this->AlicaEngineInfoSubscriber->connect();


    // Register custom structs in qt so that they can be used in slot and signal queues
    // This structure should have been previously declared as Q_DECLARE_METATYPE
    qRegisterMetaType<alica::AlicaEngineInfo>();
    qRegisterMetaType<alica::PlanTreeInfo>();

    // start the QThread, defined in base class
    start();
    this->notifyTimer->start();

}

AlicaViewerCapnzeroInterface::~AlicaViewerCapnzeroInterface()
{
    delete this->AlicaPlanTreeInfoSubscriber;
    delete this->AlicaEngineInfoSubscriber;
    zmq_ctx_term(this->ctx);
    delete this->notifyTimer;
    wait(); // defined in QThread base class
}

/* This function is run by QThread*/
void AlicaViewerCapnzeroInterface::run()
{
    Q_EMIT shutdown();
}

void AlicaViewerCapnzeroInterface::alicaEngineInfoCallback(::capnp::FlatArrayMessageReader& msg)
{
    AlicaEngineInfo aei;
    alica_capnz_msgs::AlicaEngineInfo::Reader reader = msg.getRoot<alica_capnz_msgs::AlicaEngineInfo>();

    aei.senderID = _id_manager->getIDFromBytes(reader.getSenderId().getValue().begin(), reader.getSenderId().getValue().size(),
                                                     static_cast<uint8_t>(reader.getSenderId().getType()));
    aei.masterPlan = reader.getMasterPlan();
    aei.currentPlan = reader.getCurrentPlan();
    aei.currentState = reader.getCurrentState();
    aei.currentRole = reader.getCurrentRole();
    aei.currentTask = reader.getCurrentTask();
    ::capnp::List<capnzero::ID>::Reader riwm = reader.getAgentIdsWithMe();
    for (unsigned int i = 0; i < riwm.size(); ++i) {
        capnzero::ID::Reader tmp = riwm[i];
        aei.robotIDsWithMe.push_back(_id_manager->getIDFromBytes(tmp.getValue().begin(), tmp.getValue().size(), static_cast<uint8_t>(tmp.getType())));
    }
    std::cout << "Recieved AEI: ID: " << aei.senderID << " MasterPlan: " << aei.masterPlan << " currentPlan: " << aei.currentPlan
              << " current State: " << aei.currentState << " current Role: " << aei.currentRole << " current Task: " << aei.currentTask << '\n';
    Q_EMIT alicaEngineInfoUpdate(aei);
}

void AlicaViewerCapnzeroInterface::alicaPlanInfoCallback(::capnp::FlatArrayMessageReader& msg)
{
    PlanTreeInfo pti;
    alica_capnz_msgs::PlanTreeInfo::Reader reader = msg.getRoot<alica_capnz_msgs::PlanTreeInfo>();
    pti.senderID = _id_manager->getIDFromBytes(reader.getSenderId().getValue().begin(), reader.getSenderId().getValue().size(),
                                                     static_cast<uint8_t>(reader.getSenderId().getType()));

    ::capnp::List<int64_t>::Reader stateIds = reader.getStateIds();
    for (unsigned int i = 0; i < stateIds.size(); ++i) {
        pti.stateIDs.push_back(stateIds[i]);
    }

    ::capnp::List<int64_t>::Reader succeded = reader.getSucceededEps();
    for (unsigned int i = 0; i < succeded.size(); ++i) {
        pti.succeededEPs.push_back(succeded[i]);
    }
    std::cout << "Recieved PTI senderID: " << pti.senderID << " state ids: " << pti.stateIDs.size() << " succeded eps: " << pti.succeededEPs.size() << '\n';
    Q_EMIT alicaPlanInfoUpdate(pti);
}

void AlicaViewerCapnzeroInterface::timerCallback()
{
    Q_EMIT updateTicks();
}

} // namespace alica
