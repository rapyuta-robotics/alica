#pragma once
#include "alica_msgs/AgentAnnouncementList.h"
#include "alica_msgs/AgentQueryList.h"
#include "alica_msgs/AllocationAuthorityInfoList.h"
#include "alica_msgs/PlanTreeInfoList.h"
#include "alica_msgs/SolverResultList.h"
#include "alica_msgs/SyncReadyList.h"
#include "alica_msgs/SyncTalkList.h"

#include "communication/AlicaRosCommunicationCommon.h"

#include <ros/callback_queue.h>
#include <ros/ros.h>

using namespace alica;

namespace alicaRosProxy
{

class AlicaRosEdgeBroadcastCommunication : public AlicaRosCommunicationCommon
{
public:
    AlicaRosEdgeBroadcastCommunication(const AlicaCommunicationHandlers& callbacks, ros::CallbackQueue& cb_queue = *ros::getGlobalCallbackQueue());
    virtual ~AlicaRosEdgeBroadcastCommunication();

private:
    void handleSyncReady(const alica_msgs::SyncReadyList& sr);
    void handlePlanTreeInfoRos(const alica_msgs::PlanTreeInfoList& pti);
    void handleSolverResult(const alica_msgs::SolverResultList& sr);
    void handleAgentAnnouncement(const alica_msgs::AgentAnnouncementList& pa);
    void handleAllocationAuthority(const alica_msgs::AllocationAuthorityInfoList& aai);
    void handleSyncTalk(const alica_msgs::SyncTalkList& st);
    void handleAgentQuery(const alica_msgs::AgentQueryList& pq);

    ros::Subscriber _syncReadySubscriber;
    ros::Subscriber _planTreeInfoSubscriber;
    ros::Subscriber _solverResultSubscriber;
    ros::Subscriber _presenceAnnouncementSubscriber;
    ros::Subscriber _allocationAuthorityInfoSubscriber;
    ros::Subscriber _syncTalkSubscriber;
    ros::Subscriber _presenceQuerySubscriber;
};

} /* namespace alicaRosProxy */
