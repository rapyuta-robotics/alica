/*
 * AlicaDummyCommunication.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#include "communication/AlicaDummyCommunication.h"

namespace alica_dummy_proxy {

	AlicaDummyCommunication::AlicaDummyCommunication(alica::AlicaEngine* ae):
					alica::IAlicaCommunication(ae)
	{
		// TODO Auto-generated constructor stub

	}

	AlicaDummyCommunication::~AlicaDummyCommunication()
	{
		// TODO Auto-generated destructor stub
	}
	void AlicaDummyCommunication::sendAllocationAuthority(alica::AllocationAuthorityInfo& aai)
	{

	}
	void AlicaDummyCommunication::sendBehaviourEngineInfo(alica::BehaviourEngineInfo& bi)
	{

	}
	void AlicaDummyCommunication::sendPlanTreeInfo(alica::PlanTreeInfo& pti)
	{

	}
	void AlicaDummyCommunication::sendRoleSwitch(alica::RoleSwitch& rs)
	{

	}
	void AlicaDummyCommunication::sendSyncReady(alica::SyncReady& sr)
	{

	}
	void AlicaDummyCommunication::sendSyncTalk(alica::SyncTalk& st)
	{

	}
	void AlicaDummyCommunication::sendSolverResult(alica::SolverResult& sr)
	{

	}

	void AlicaDummyCommunication::tick()
	{

	}
	void AlicaDummyCommunication::startCommunication()
	{

	}
	void AlicaDummyCommunication::stopCommunication()
	{

	}

} /* namespace alica */
