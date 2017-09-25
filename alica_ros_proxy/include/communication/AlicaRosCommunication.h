#pragma once

#include "engine/IAlicaCommunication.h"
#include "ros/ros.h"

#include "alica_ros_proxy/AllocationAuthorityInfo.h"
#include "alica_ros_proxy/AlicaEngineInfo.h"
#include "alica_ros_proxy/PlanTreeInfo.h"
#include "alica_ros_proxy/RoleSwitch.h"
#include "alica_ros_proxy/SyncReady.h"
#include "alica_ros_proxy/SyncTalk.h"
#include "alica_ros_proxy/SolverResult.h"

using namespace alica;

namespace alicaRosProxy
{

	class AlicaRosCommunication : public alica::IAlicaCommunication
	{
	public:
		AlicaRosCommunication(AlicaEngine* ae);
		virtual ~AlicaRosCommunication();

		virtual void tick();

		virtual void sendAllocationAuthority(AllocationAuthorityInfo& aai) const;
		virtual void sendAlicaEngineInfo(AlicaEngineInfo& bi) const;
		virtual void sendPlanTreeInfo(PlanTreeInfo& pti) const;
		virtual void sendRoleSwitch(RoleSwitch& rs) const;
		virtual void sendSyncReady(SyncReady& sr) const;
		virtual void sendSyncTalk(SyncTalk& st) const;
		virtual void sendSolverResult(SolverResult& sr) const;
		virtual void sendLogMessage(int level, string& message) const;

		virtual void handleAllocationAuthorityRos(alica_ros_proxy::AllocationAuthorityInfoPtr aai);
		virtual void handlePlanTreeInfoRos(alica_ros_proxy::PlanTreeInfoPtr pti);
		virtual void handleSyncReadyRos(alica_ros_proxy::SyncReadyPtr sr);
		virtual void handleSyncTalkRos(alica_ros_proxy::SyncTalkPtr st);
		virtual void handleSolverResult(alica_ros_proxy::SolverResultPtr sr);

		virtual void startCommunication();
		virtual void stopCommunication();

	protected:
		ros::NodeHandle* rosNode;
		ros::AsyncSpinner* spinner;

		ros::Publisher AlicaEngineInfoPublisher;
		ros::Publisher RoleSwitchPublisher;

		ros::Publisher AllocationAuthorityInfoPublisher;
		ros::Subscriber AllocationAuthorityInfoSubscriber;

		ros::Publisher PlanTreeInfoPublisher;
		ros::Subscriber PlanTreeInfoSubscriber;

		ros::Publisher SyncReadyPublisher;
		ros::Subscriber SyncReadySubscriber;

		ros::Publisher SyncTalkPublisher;
		ros::Subscriber SyncTalkSubscriber;

		ros::Publisher SolverResultPublisher;
		ros::Subscriber SolverResultSubscriber;

		string allocationAuthorityInfoTopic;
		string ownRoleTopic;
		string alicaEngineInfoTopic;
		string planTreeInfoTopic;
		string syncReadyTopic;
		string syncTalkTopic;
		string solverResultTopic;

		bool isRunning;

		supplementary::SystemConfig* sc;
	};

} /* namespace alicaRosProxy */
