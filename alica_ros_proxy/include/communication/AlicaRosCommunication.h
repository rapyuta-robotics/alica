/*
 * AlicaRosCommunication.h
 *
 *  Created on: 10.09.2014
 *      Author: endy
 */

#ifndef ALICAROSCOMMUNICATION_H_
#define ALICAROSCOMMUNICATION_H_

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

		virtual void sendAllocationAuthority(AllocationAuthorityInfo& aai);
		virtual void sendAlicaEngineInfo(AlicaEngineInfo& bi);
		virtual void sendPlanTreeInfo(PlanTreeInfo& pti);
		virtual void sendRoleSwitch(RoleSwitch& rs);
		virtual void sendSyncReady(SyncReady& sr);
		virtual void sendSyncTalk(SyncTalk& st);
		virtual void sendSolverResult(SolverResult& sr);

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

#endif /* ALICAROSCOMMUNICATION_H_ */
