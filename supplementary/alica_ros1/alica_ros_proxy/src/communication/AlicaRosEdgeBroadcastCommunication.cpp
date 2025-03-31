#include "communication/AlicaRosEdgeBroadcastCommunication.h"

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
const std::string planTreeInfoTopic = "/edge_broadcaster/PlanTreeInfos";
const std::string solverResultTopic = "/edge_broadcaster/SolverResults";
const std::string presenceAnnouncementTopic = "/edge_broadcaster/AgentAnnouncements";
const std::string allocationAuthorityInfoTopic = "/edge_broadcaster/AllocationAuthorityInfos";
const std::string syncReadyTopic = "/edge_broadcaster/SyncReadies";
const std::string syncTalkTopic = "/edge_broadcaster/SyncTalks";
const std::string presenceQueryTopic = "/edge_broadcaster/AgentQueries";
} // namespace

AlicaRosEdgeBroadcastCommunication::AlicaRosEdgeBroadcastCommunication(const AlicaCommunicationHandlers& callbacks, ros::CallbackQueue& cb_queue)
        : AlicaRosCommunicationCommon(callbacks, cb_queue)
        , _nh()
{
    _planTreeInfoSubscriber =
            _nh.subscribe(planTreeInfoTopic, 5, &AlicaRosEdgeBroadcastCommunication::handlePlanTreeInfoRos, (AlicaRosEdgeBroadcastCommunication*) this);
    _solverResultSubscriber =
            _nh.subscribe(solverResultTopic, 5, &AlicaRosEdgeBroadcastCommunication::handleSolverResult, (AlicaRosEdgeBroadcastCommunication*) this);
    _presenceAnnouncementSubscriber = _nh.subscribe(
            presenceAnnouncementTopic, 50, &AlicaRosEdgeBroadcastCommunication::handleAgentAnnouncement, (AlicaRosEdgeBroadcastCommunication*) this);
    _allocationAuthorityInfoSubscriber = _nh.subscribe(
            allocationAuthorityInfoTopic, 10, &AlicaRosEdgeBroadcastCommunication::handleAllocationAuthority, (AlicaRosEdgeBroadcastCommunication*) this);
    _syncReadySubscriber = _nh.subscribe(syncReadyTopic, 5, &AlicaRosEdgeBroadcastCommunication::handleSyncReady, (AlicaRosEdgeBroadcastCommunication*) this);
    _syncTalkSubscriber = _nh.subscribe(syncTalkTopic, 5, &AlicaRosEdgeBroadcastCommunication::handleSyncTalk, (AlicaRosEdgeBroadcastCommunication*) this);
    _presenceQuerySubscriber =
            _nh.subscribe(presenceQueryTopic, 5, &AlicaRosEdgeBroadcastCommunication::handleAgentQuery, (AlicaRosEdgeBroadcastCommunication*) this);
}

AlicaRosEdgeBroadcastCommunication::~AlicaRosEdgeBroadcastCommunication()
{
    _nh.shutdown();
}

void AlicaRosEdgeBroadcastCommunication::handleAgentAnnouncement(const alica_msgs::AgentAnnouncementList& pa)
{
    for (const auto& agent_announcement : pa.agent_announcements) {
        AlicaRosCommunicationCommon::handleAgentAnnouncement(agent_announcement);
    }
}

void AlicaRosEdgeBroadcastCommunication::handlePlanTreeInfoRos(const alica_msgs::PlanTreeInfoList& pti)
{
    for (const auto& info : pti.plan_tree_infos) {
        auto info_ptr = alica_msgs::PlanTreeInfoPtr(new alica_msgs::PlanTreeInfo(info));
        AlicaRosCommunicationCommon::handlePlanTreeInfoRos(info_ptr);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleSolverResult(const alica_msgs::SolverResultList& sr)
{
    for (const auto& result : sr.solver_results) {
        AlicaRosCommunicationCommon::handleSolverResult(result);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleSyncReady(const alica_msgs::SyncReadyList& sr)
{
    for (const auto& sync_ready : sr.sync_readies) {
        auto sync_ready_ptr = alica_msgs::SyncReadyPtr(new alica_msgs::SyncReady(sync_ready));
        AlicaRosCommunicationCommon::handleSyncReadyRos(sync_ready_ptr);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleAllocationAuthority(const alica_msgs::AllocationAuthorityInfoList& aai)
{
    for (const auto& allocation_authority : aai.allocation_authority_infos) {
        AlicaRosCommunicationCommon::handleAllocationAuthorityRos(allocation_authority);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleSyncTalk(const alica_msgs::SyncTalkList& st)
{
    for (const auto& sync_talk : st.sync_talks) {
        auto sync_talk_ptr = alica_msgs::SyncTalkPtr(new alica_msgs::SyncTalk(sync_talk));
        AlicaRosCommunicationCommon::handleSyncTalkRos(sync_talk_ptr);
    }
}

void AlicaRosEdgeBroadcastCommunication::handleAgentQuery(const alica_msgs::AgentQueryList& pq)
{
    for (const auto& agent_query : pq.agent_queries) {
        AlicaRosCommunicationCommon::handleAgentQuery(agent_query);
    }
}
} /* namespace alicaRosProxy */
