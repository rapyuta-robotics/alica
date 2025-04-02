#include "communication/AlicaRosCommunication.h"

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

using namespace alica;

namespace alicaRosProxy
{

using std::make_shared;
using std::string;

namespace
{
const std::string planTreeInfoTopic = "/AlicaEngine/PlanTreeInfo";
const std::string solverResultTopic = "/AlicaEngine/SolverResult";
const std::string presenceAnnouncementTopic = "/AlicaEngine/AgentAnnouncement";
const std::string allocationAuthorityInfoTopic = "/AlicaEngine/AllocationAuthorityInfo";
const std::string syncReadyTopic = "/AlicaEngine/SyncReady";
const std::string syncTalkTopic = "/AlicaEngine/SyncTalk";
const std::string presenceQueryTopic = "/AlicaEngine/AgentQuery";

} // namespace

AlicaRosCommunication::AlicaRosCommunication(const AlicaCommunicationHandlers& callbacks, ros::CallbackQueue& cb_queue)
        : AlicaRosCommunicationCommon(callbacks, cb_queue)
{

    _planTreeInfoSubscriber = _nh.subscribe(planTreeInfoTopic, 5, &AlicaRosCommunicationCommon::handlePlanTreeInfoRos, (AlicaRosCommunicationCommon*) this);
    _solverResultSubscriber = _nh.subscribe(solverResultTopic, 5, &AlicaRosCommunicationCommon::handleSolverResult, (AlicaRosCommunicationCommon*) this);
    _presenceAnnouncementSubscriber =
            _nh.subscribe(presenceAnnouncementTopic, 50, &AlicaRosCommunicationCommon::handleAgentAnnouncement, (AlicaRosCommunicationCommon*) this);
    _allocationAuthorityInfoSubscriber =
            _nh.subscribe(allocationAuthorityInfoTopic, 10, &AlicaRosCommunicationCommon::handleAllocationAuthorityRos, (AlicaRosCommunicationCommon*) this);
    _syncReadySubscriber = _nh.subscribe(syncReadyTopic, 5, &AlicaRosCommunicationCommon::handleSyncReadyRos, (AlicaRosCommunicationCommon*) this);
    _syncTalkSubscriber = _nh.subscribe(syncTalkTopic, 5, &AlicaRosCommunicationCommon::handleSyncTalkRos, (AlicaRosCommunicationCommon*) this);
    _presenceQuerySubscriber = _nh.subscribe(presenceQueryTopic, 5, &AlicaRosCommunicationCommon::handleAgentQuery, (AlicaRosCommunicationCommon*) this);
}

AlicaRosCommunication::~AlicaRosCommunication()
{
    _nh.shutdown();
}

} /* namespace alicaRosProxy */
