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

	void AlicaRosCommunication::sendAllocationAuthority(AllocationAuthorityInfo& aai)
	{
		alica_ros_proxy::AllocationAuthorityInfo aais;
		for (int i = 0; i < aai.senderID.getSize(); i++)
		{
			aais.senderID.id.push_back(*(aai.senderID.getRaw()+i));
		}
		aais.planID = aai.planId;
		aais.parentState = aai.parentState;
		aais.planType = aai.planType;
		for (int i = 0; i < aai.authority.getSize(); i++)
		{
			aais.authority.id.push_back(*(aai.authority.getRaw()+i));
		}
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
		for (int i : bi.robotIDsWithMe)
		{
			bis.robotIDsWithMe.push_back(i);
		}
		bis.senderID = bi.senderID;

		if (this->isRunning)
		{
			this->AlicaEngineInfoPublisher.publish(bis);
		}
	}

	void AlicaRosCommunication::sendPlanTreeInfo(PlanTreeInfo& pti)
	{
		alica_ros_proxy::PlanTreeInfo ptis;
		ptis.senderID = pti.senderID.getRaw();
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
		rss.senderID = rs.roleID;

		if (this->isRunning)
		{
			this->RoleSwitchPublisher.publish(rss);
		}
	}

	void AlicaRosCommunication::sendSyncReady(SyncReady& sr)
	{
		alica_ros_proxy::SyncReady srs;

		srs.senderID = sr.senderID.getRaw();
		srs.syncTransitionID = sr.syncTransitionID;

		if (this->isRunning)
		{
			this->SyncReadyPublisher.publish(srs);
		}
	}

	void AlicaRosCommunication::sendSyncTalk(SyncTalk& st)
	{
		alica_ros_proxy::SyncTalk sts;

		sts.senderID = st.senderID.getRaw();
		for (auto sd : st.syncData)
		{
			alica_ros_proxy::SyncData sds;
			sds.ack = sd->ack;
			sds.conditionHolds = sd->conditionHolds;
			sds.robotID = sd->robotID;
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

		srs.senderID = sr.senderID.getRaw();
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
		aaiPtr->senderID = this->ae->getRobotIDFactory()->create(aai->senderID.data(), aai->senderID.size());
		aaiPtr->planId = aai->planID;
		aaiPtr->parentState = aai->parentState;
		aaiPtr->planType = aai->planType;
		aaiPtr->authority = aai->authority;
		for (auto &ep : aai->entrypoints)
		{
			alica::EntryPointRobots newEP;
			newEP.entrypoint = ep.entryPoint;
			newEP.robots = ep.robots;

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
		ptiPtr->senderID = pti->senderID;
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

		srPtr->senderID = sr->senderID;
		srPtr->syncTransitionID = sr->syncTransitionID;

		if (this->isRunning)
		{
			this->onSyncReadyReceived(srPtr);
		}
	}

	void AlicaRosCommunication::handleSyncTalkRos(alica_ros_proxy::SyncTalkPtr st)
	{
		auto stPtr = make_shared<SyncTalk>();

		stPtr->senderID = st->senderID;
		for (auto &sd : st->syncData)
		{
			SyncData* sds = new SyncData();
			sds->ack = sd.ack;
			sds->conditionHolds = sd.conditionHolds;
			sds->robotID = sd.robotID;
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

		srPtr->senderID = sr->senderID;
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
