#include "alica_viewer/alica_viewer_capnzero_interface.h"

#include <alica_capnzero_proxy/ContainerUtils.h>
#include <alica_msg/AlicaEngineInfo.capnp.h>
#include <alica_msg/PlanTreeInfo.capnp.h>

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

    this->AlicaPlanTreeInfoSubscriber = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->AlicaPlanTreeInfoSubscriber->setTopic(this->planTreeInfoTopic);
    this->AlicaEngineInfoSubscriber = new capnzero::Subscriber(this->ctx, capnzero::Protocol::UDP);
    this->AlicaPlanTreeInfoSubscriber->setTopic(this->alicaEngineInfoTopic);

    this->AlicaPlanTreeInfoSubscriber->addAddress(this->url);
    this->AlicaEngineInfoSubscriber->addAddress(this->url);


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
    AlicaEngineInfo aei = alica_capnzero_proxy::ContainerUtils::toAlicaEngineInfo(msg, _id_manager);
    std::cout << "Recieved AEI: ID: " << aei.senderID << " MasterPlan: " << aei.masterPlan << " currentPlan: " << aei.currentPlan
              << " current State: " << aei.currentState << " current Role: " << aei.currentRole << " current Task: " << aei.currentTask << '\n';
    Q_EMIT alicaEngineInfoUpdate(aei);
}

void AlicaViewerCapnzeroInterface::alicaPlanInfoCallback(::capnp::FlatArrayMessageReader& msg)
{
    PlanTreeInfo pti = alica_capnzero_proxy::ContainerUtils::toPlanTreeInfo(msg, _id_manager);
    std::cout << "Recieved PTI senderID: " << pti.senderID << " state ids: " << pti.stateIDs.size() << " succeded eps: " << pti.succeededEPs.size() << '\n';
    Q_EMIT alicaPlanInfoUpdate(pti);
}

void AlicaViewerCapnzeroInterface::timerCallback()
{
    Q_EMIT updateTicks();
}

} // namespace alica
