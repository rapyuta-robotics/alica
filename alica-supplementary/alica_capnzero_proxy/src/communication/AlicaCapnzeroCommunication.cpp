#include "communication/AlicaCapnzeroCommunication.h"

// Alica messages includes. They will be replaced with capnproto messages!
#include "alica_msgs/AlicaEngineInfo.h"
#include "alica_msgs/AllocationAuthorityInfo.h"
#include "alica_msgs/PlanTreeInfo.h"
#include "alica_msgs/RoleSwitch.h"
#include "alica_msgs/SolverResult.h"
#include "alica_msgs/SyncReady.h"
#include "alica_msgs/SyncTalk.h"

#include <capnzero/CapnZero.h>
#include <capnp/common.h>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include <kj/array.h>

using namespace alica;

namespace alicaCapnzeroProxy
{
    using std::make_shared;
    using std::string;

    AlicaCapnzeroCommunication::AlicaCapnzeroCommunication(AlicaEngine* ae) : IAlicaCommunication(ae)
    {
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
//        this->AlicaEngineInfoPublisher->bind(capnzero::CommType::UDP, this->url);
//        this->RoleSwitchPublisher->bind(capnzero::CommType::UDP, this->url);

        // Setup Subscribers:
        this->AlicaEngineInfoSubscriber = new capnzero::Subscriber(this->ctx, this->alicaEngineInfoTopic);
        this->RoleSwitchSubscriber = new capnzero::Subscriber(this->ctx, this->ownRoleTopic);
        this->AllocationAuthorityInfoSubscriber = new capnzero::Subscriber(this->ctx, this->allocationAuthorityInfoTopic);
        this->PlanTreeInfoSubscriber = new capnzero::Subscriber(this->ctx, this->planTreeInfoTopic);
        this->SyncReadySubscriber = new capnzero::Subscriber(this->ctx, this->syncReadyTopic);
        this->SyncTalkSubscriber = new capnzero::Subscriber(this->ctx, this->syncTalkTopic);
        this->SolverResultSubscriber = new capnzero::Subscriber(this->ctx, this->solverResultTopic);
    }
    AlicaCapnzeroCommunication::~AlicaCapnzeroCommunication()
    {
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

    void AlicaCapnzeroCommunication::sendAllocationAuthority(const AllocationAuthorityInfo& aai) const
    {
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
}