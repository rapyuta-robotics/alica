/*
 * AlicaRosCommunication.cpp
 *
 *  Created on: 10.09.2014
 *      Author: endy
 */

#include "communication/AlicaRosCommunication.h"

#include "engine/IRobotIDFactory.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/containers/SolverResult.h"
#include "engine/containers/SolverVar.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/containers/SyncData.h"
#include "engine/teamobserver/TeamObserver.h"

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <SystemConfig.h>
#include <Configuration.h>
#include <engine/containers/AlicaEngineInfo.h>
#include <ros/console.h>

using namespace alica;

namespace alicaRosProxy
{

	AlicaRosCommunication::AlicaRosCommunication(AlicaEngine* ae) :
			IAlicaCommunication(ae)
	{
		this->isRunning = false;
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);

		// read topic strings from AlicaRosProxy.conf
		this->sc = supplementary::SystemConfig::getInstance();
		this->allocationAuthorityInfoTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.allocationAuthorityInfoTopic", NULL);
		this->ownRoleTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.ownRoleTopic", NULL);
		this->alicaEngineInfoTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.alicaEngineInfoTopic", NULL);
		this->planTreeInfoTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.planTreeInfoTopic", NULL);
		this->syncReadyTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.syncReadyTopic", NULL);
		this->syncTalkTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.syncTalkTopic", NULL);
		this->solverResultTopic = (*sc)["AlicaRosProxy"]->get<string>("Topics.solverResultTopic", NULL);

		AllocationAuthorityInfoPublisher = rosNode->advertise<alica_ros_proxy::AllocationAuthorityInfo>(
				this->allocationAuthorityInfoTopic, 2);
		AllocationAuthorityInfoSubscriber = rosNode->subscribe(this->allocationAuthorityInfoTopic, 10,
																&AlicaRosCommunication::handleAllocationAuthorityRos,
																(AlicaRosCommunication*)this);

		AlicaEngineInfoPublisher = rosNode->advertise<alica_ros_proxy::AlicaEngineInfo>(this->alicaEngineInfoTopic, 2);
		RoleSwitchPublisher = rosNode->advertise<alica_ros_proxy::RoleSwitch>(this->ownRoleTopic, 10);

		PlanTreeInfoPublisher = rosNode->advertise<alica_ros_proxy::PlanTreeInfo>(this->planTreeInfoTopic, 10);
		PlanTreeInfoSubscriber = rosNode->subscribe(this->planTreeInfoTopic, 1,
													&AlicaRosCommunication::handlePlanTreeInfoRos,
													(AlicaRosCommunication*)this);

		SyncReadyPublisher = rosNode->advertise<alica_ros_proxy::SyncReady>(this->syncReadyTopic, 10);
		SyncReadySubscriber = rosNode->subscribe(this->syncReadyTopic, 5, &AlicaRosCommunication::handleSyncReadyRos,
													(AlicaRosCommunication*)this);
		SyncTalkPublisher = rosNode->advertise<alica_ros_proxy::SyncTalk>(this->syncTalkTopic, 10);
		SyncTalkSubscriber = rosNode->subscribe(this->syncTalkTopic, 5, &AlicaRosCommunication::handleSyncTalkRos,
												(AlicaRosCommunication*)this);

		SolverResultPublisher = rosNode->advertise<alica_ros_proxy::SolverResult>(this->solverResultTopic, 10);
		SolverResultSubscriber = rosNode->subscribe(this->solverResultTopic, 5,
													&AlicaRosCommunication::handleSolverResult,
													(AlicaRosCommunication*)this);

	}

	AlicaRosCommunication::~AlicaRosCommunication()
	{
		if (this->isRunning)
		{
			spinner->stop();
		}
		delete spinner;

		AllocationAuthorityInfoSubscriber.shutdown();
		RoleSwitchPublisher.shutdown();
		PlanTreeInfoSubscriber.shutdown();
		SyncReadySubscriber.shutdown();
		SyncTalkSubscriber.shutdown();
		rosNode->shutdown();
		delete rosNode;

	}

	void AlicaRosCommunication::tick()
	{
		if (this->isRunning)
		{
			//Use this for synchronous communication!
			//ros::spinOnce();
		}
	}

	void AlicaRosCommunication::convertToRosID(alica::IRobotID& robotID, std::vector<uint8_t>& robotRosID)
	{
		for (int i = 0; i < robotID.getSize(); i++)
		{
			robotRosID.push_back(*(robotID.getRaw() + i));
		}
	}

	alica::IRobotID AlicaRosCommunication::convertToAlicaID( std::vector<uint8_t>& robotRosID)
	{
		unsigned char* _robotRosID = reinterpret_cast<unsigned char*>(robotRosID.data());
		return this->ae->getRobotIDFactory()->create(_robotRosID, sizeof(robotRosID));
	}


	void AlicaRosCommunication::sendAllocationAuthority(AllocationAuthorityInfo& aai)
	{
		alica_ros_proxy::AllocationAuthorityInfo aais;

		convertToRosID(aai.senderID, aais.senderID.id);
		
		aais.planID = aai.planId;
		aais.parentState = aai.parentState;
		aais.planType = aai.planType;
		
		convertToRosID(aai.authority, aais.authority.id);

		for (auto &ep : aai.entryPointRobots)
		{
			alica_ros_proxy::EntryPointRobots newEP;
			newEP.entryPoint = ep.entrypoint;
			int i = 0;
			for (auto& robotId : ep.robots)
			{
				newEP.robots.push_back(alica_ros_proxy::EntryPointRobots::_robots_type::value_type());
				for (int j = 0; j < robotId.getSize(); j++)
				{
					newEP.robots[i].id.push_back(*(robotId.getRaw()+j));
				}
				i++;
			}
			aais.entrypoints.push_back(newEP);
		}

		if (this->isRunning)
		{
			this->AllocationAuthorityInfoPublisher.publish(aais);
		}
	}

	void AlicaRosCommunication::sendAlicaEngineInfo(AlicaEngineInfo& bi)
	{
		alica_ros_proxy::AlicaEngineInfo bis;
		bis.currentPlan = bi.currentPlan;
		bis.currentRole = bi.currentRole;
		bis.currentState = bi.currentState;
		bis.currentTask = bi.currentTask;
		bis.masterPlan = bi.masterPlan;

		for (alica::IRobotID robotID : bi.robotIDsWithMe)
		{

			alica_ros_proxy::AllocationAuthorityInfo::_senderID_type rosRobotID;
			convertToRosID(robotID, rosRobotID.id);
			bis.robotIDsWithMe.push_back(rosRobotID);
		}

		convertToRosID(bi.senderID, bis.senderID.id);

		if (this->isRunning)
		{
			this->AlicaEngineInfoPublisher.publish(bis);
		}
	}

	void AlicaRosCommunication::sendPlanTreeInfo(PlanTreeInfo& pti)
	{
		alica_ros_proxy::PlanTreeInfo ptis;
		convertToRosID(pti.senderID, ptis.senderID.id);
		for (long i : pti.stateIDs)
		{
			ptis.stateIDs.push_back(i);
		}
		for (long i : pti.succeededEPs)
		{
			ptis.succeededEps.push_back(i);
		}
		if (this->isRunning)
		{
			this->PlanTreeInfoPublisher.publish(ptis);
		}
	}

	void AlicaRosCommunication::sendRoleSwitch(RoleSwitch& rs)
	{
		alica_ros_proxy::RoleSwitch rss;

		rss.roleID = rs.roleID;
		alica::IRobotID robotID = this->ae->getTeamObserver()->getOwnId();
		convertToRosID(robotID, rss.senderID.id);

		if (this->isRunning)
		{
			this->RoleSwitchPublisher.publish(rss);
		}
	}

	void AlicaRosCommunication::sendSyncReady(SyncReady& sr)
	{
		alica_ros_proxy::SyncReady srs;

		convertToRosID(sr.senderID, srs.senderID.id);
		srs.syncTransitionID = sr.syncTransitionID;

		if (this->isRunning)
		{
			this->SyncReadyPublisher.publish(srs);
		}
	}

	void AlicaRosCommunication::sendSyncTalk(SyncTalk& st)
	{
		alica_ros_proxy::SyncTalk sts;
		convertToRosID(st.senderID, sts.senderID.id);

		for (auto sd : st.syncData)
		{
			alica_ros_proxy::SyncData sds;
			sds.ack = sd->ack;
			sds.conditionHolds = sd->conditionHolds;
			convertToRosID(sd->robotID,sds.robotID.id);
			sds.transitionID = sd->transitionID;
			sts.syncData.push_back(sds);
		}

		if (this->isRunning)
		{
			this->SyncTalkPublisher.publish(sts);
		}
	}

	void AlicaRosCommunication::sendSolverResult(SolverResult& sr)
	{
		alica_ros_proxy::SolverResult srs;
		convertToRosID(sr.senderID, srs.senderID.id);

		for (auto sv : sr.vars)
		{
			alica_ros_proxy::SolverVar svs;
			svs.id = sv->id;
			svs.value = sv->value;
			srs.vars.push_back(svs);
		}

		if (this->isRunning)
		{
			this->SolverResultPublisher.publish(srs);
		}
	}


	void AlicaRosCommunication::handleAllocationAuthorityRos(alica_ros_proxy::AllocationAuthorityInfoPtr aai)
	{
		auto aaiPtr = make_shared<AllocationAuthorityInfo>();
		aaiPtr->senderID = convertToAlicaID(aai->senderID.id);
		aaiPtr->planId = aai->planID;
		aaiPtr->parentState = aai->parentState;
		aaiPtr->planType = aai->planType;
		aaiPtr->authority = convertToAlicaID(aai->authority.id);

		for (auto &ep : aai->entrypoints)
		{
			alica::EntryPointRobots newEP;
			newEP.entrypoint = ep.entryPoint;

			for (auto robotID : ep.robots)
			{
				newEP.robots.push_back(convertToAlicaID(robotID.id));
			}

			aaiPtr->entryPointRobots.push_back(newEP);
		}

		if (this->isRunning)
		{
			this->onAuthorityInfoReceived(aaiPtr);
		}
	}

	void AlicaRosCommunication::handlePlanTreeInfoRos(alica_ros_proxy::PlanTreeInfoPtr pti)
	{
		auto ptiPtr = make_shared<PlanTreeInfo>();
		ptiPtr->senderID = convertToAlicaID(pti->senderID.id);
		for (long i : pti->stateIDs)
		{
			ptiPtr->stateIDs.push_back(i);
		}
		for (long i : pti->succeededEps)
		{
			ptiPtr->succeededEPs.push_back(i);
		}

		if (this->isRunning)
		{
			this->onPlanTreeInfoReceived(ptiPtr);
		}
	}

	void AlicaRosCommunication::handleSyncReadyRos(alica_ros_proxy::SyncReadyPtr sr)
	{
		auto srPtr = make_shared<SyncReady>();
		srPtr->senderID = convertToAlicaID(sr->senderID.id);
		srPtr->syncTransitionID = sr->syncTransitionID;

		if (this->isRunning)
		{
			this->onSyncReadyReceived(srPtr);
		}
	}

	void AlicaRosCommunication::handleSyncTalkRos(alica_ros_proxy::SyncTalkPtr st)
	{
		auto stPtr = make_shared<SyncTalk>();
		//stPtr->senderID = st->senderID;
		stPtr->senderID = convertToAlicaID(st->senderID.id);

		for (auto &sd : st->syncData)
		{
			SyncData* sds = new SyncData();
			sds->ack = sd.ack;
			sds->conditionHolds = sd.conditionHolds;
			sds->robotID = convertToAlicaID(sd.robotID.id);
			sds->transitionID = sd.transitionID;
			stPtr->syncData.push_back(sds);
		}

		if (this->isRunning)
		{
			this->onSyncTalkReceived(stPtr);
		}
	}

	void AlicaRosCommunication::handleSolverResult(alica_ros_proxy::SolverResultPtr sr)
	{
		auto srPtr = make_shared<SolverResult>();
		srPtr->senderID = convertToAlicaID(sr->senderID.id);

		for (auto &sv : sr->vars)
		{
			SolverVar* svs = new SolverVar();
			svs->id = sv.id;
			svs->value = sv.value;
			srPtr->vars.push_back(svs);
		}

		if (this->isRunning)
		{
			this->onSolverResult(srPtr);
		}
	}

	void AlicaRosCommunication::sendLogMessage(int level, string& message) {
		switch(level) {
			case ::ros::console::levels::Debug:
				ROS_DEBUG("AlicaMessage: %s", message.c_str());
				break;
			case ::ros::console::levels::Info:
				ROS_INFO("AlicaMessage: %s", message.c_str());
				break;
			case ::ros::console::levels::Warn:
				ROS_WARN("AlicaMessage: %s", message.c_str());
				break;
			case ::ros::console::levels::Error:
				ROS_ERROR("AlicaMessage: %s", message.c_str());
				break;
			case ::ros::console::levels::Fatal:
				ROS_FATAL("AlicaMessage: %s", message.c_str());
				break;
			default:
				ROS_ERROR("AlicaMessage: %s", message.c_str());
				break;
		}

	}


	void AlicaRosCommunication::startCommunication()
	{
		this->isRunning = true;
		spinner->start();
	}
	void AlicaRosCommunication::stopCommunication()
	{
		this->isRunning = false;
		spinner->stop();
	}

} /* namespace alicaRosProxy */
