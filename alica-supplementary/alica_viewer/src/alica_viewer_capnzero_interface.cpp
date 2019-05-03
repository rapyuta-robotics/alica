#include "alica_viewer/alica_viewer_capnzero_interface.h"
#include <msgs/AlicaEngineInfo.capnp.h>
#include <msgs/PlanTreeInfo.capnp.h>

#include <capnzero/Subscriber.h>
#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>

namespace alica
{

AlicaViewerCapnzeroInterface::AlicaViewerCapnzeroInterface(int argc, char* argv[])
    : _agent_id_manager(new essentials::AgentIDManager(new essentials::AgentIDFactory()))
{
    // Create zmq context
    this->ctx = zmq_ctx_new();
    this->url = "224.0.0.2:5555";

    this->alicaEngineInfoTopic = "EngineInfo";
    this->planTreeInfoTopic = "planTree";

    this->AlicaPlanTreeInfoSubscriber = new capnzero::Subscriber(this->ctx, this->planTreeInfoTopic);
    this->AlicaEngineInfoSubscriber = new capnzero::Subscriber(this->ctx, this->alicaEngineInfoTopic);

    this->AlicaPlanTreeInfoSubscriber->connect(capnzero::CommType::UDP, this->url);
    this->AlicaEngineInfoSubscriber->connect(capnzero::CommType::UDP, this->url);

    this->AlicaPlanTreeInfoSubscriber->subscribe(&AlicaViewerCapnzeroInterface::alicaPlanInfoCallback, &(*this));
    this->AlicaEngineInfoSubscriber->subscribe(&AlicaViewerCapnzeroInterface::alicaEngineInfoCallback, &(*this));

    // Register custom structs in qt so that they can be used in slot and signal queues
    // This structure should have been previously declared as Q_DECLARE_METATYPE
    qRegisterMetaType<alica::AlicaEngineInfo>();
    qRegisterMetaType<alica::PlanTreeInfo>();

    // start the QThread, defined in base class
    start();
}

AlicaViewerCapnzeroInterface::~AlicaViewerCapnzeroInterface()
{
    delete this->AlicaPlanTreeInfoSubscriber;
    delete this->AlicaEngineInfoSubscriber;
    zmq_ctx_term(this->ctx);
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
    alica_capnp_msgs::AlicaEngineInfo::Reader reader = msg.getRoot<alica_capnp_msgs::AlicaEngineInfo>();
    std::vector<uint8_t> id;
    id.assign(reader.getSenderId().getValue().begin(), reader.getSenderId().getValue().end());
    aei.senderID = _agent_id_manager->getIDFromBytes(id);
    id.clear();
    aei.masterPlan = reader.getMasterPlan();
    aei.currentPlan = reader.getCurrentPlan();
    aei.currentState = reader.getCurrentState();
    aei.currentRole = reader.getCurrentRole();
    aei.currentTask = reader.getCurrentTask();
    ::capnp::List<UUID>::Reader riwm = reader.getAgentIdsWithMe();
    for (unsigned int i = 0; i < riwm.size(); ++i) {
        UUID::Reader tmp = riwm[i];
        id.assign(tmp.getValue().begin(), tmp.getValue().end());
        aei.robotIDsWithMe.push_back(_agent_id_manager->getIDFromBytes(id));
        id.clear();
    }
    Q_EMIT alicaEngineInfoUpdate(aei);
}

void AlicaViewerCapnzeroInterface::alicaPlanInfoCallback(::capnp::FlatArrayMessageReader& msg)
{
    PlanTreeInfo pti;
    alica_capnp_msgs::PlanTreeInfo::Reader reader = msg.getRoot<alica_capnp_msgs::PlanTreeInfo>();
    std::vector<uint8_t> id;
    id.assign(reader.getSenderId().getValue().begin(), reader.getSenderId().getValue().end());
    pti.senderID = _agent_id_manager->getIDFromBytes(id);
    id.clear();

    ::capnp::List<int64_t>::Reader stateIds = reader.getStateIds();
    for (unsigned int i = 0; i < stateIds.size(); ++i) {
        pti.stateIDs.push_back(stateIds[i]);
    }

    ::capnp::List<int64_t>::Reader succeded = reader.getSucceededEps();
    for (unsigned int i = 0; i < succeded.size(); ++i) {
        pti.succeededEPs.push_back(succeded[i]);
    }
    Q_EMIT alicaPlanInfoUpdate(pti);
}

} // namespace alica