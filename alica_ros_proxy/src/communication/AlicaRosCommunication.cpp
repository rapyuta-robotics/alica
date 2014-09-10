/*
 * AlicaRosCommunication.cpp
 *
 *  Created on: 10.09.2014
 *      Author: endy
 */

#include "communication/AlicaRosCommunication.h"

#include "engine/containers/AllocationAuthorityInfo.h"
#include "engine/containers/BehaviourEngineInfo.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"

using namespace alica;

namespace alicaRosProxy
{

	AlicaRosCommunication::AlicaRosCommunication(AlicaEngine* ae) : IAlicaCommunication(ae)
	{
		// TODO Auto-generated constructor stub

	}

	AlicaRosCommunication::~AlicaRosCommunication()
	{
		// TODO Auto-generated destructor stub
	}

	void AlicaRosCommunication::tick()
	{
	}

	void AlicaRosCommunication::sendAllocationAuthority(AllocationAuthorityInfo aai)
	{
	}

	void AlicaRosCommunication::sendBehaviourEngineInfo(BehaviourEngineInfo bi)
	{
	}

	void AlicaRosCommunication::sendPlanTreeInfo(PlanTreeInfo pti)
	{
	}

	void AlicaRosCommunication::sendRoleSwitch(RoleSwitch rs)
	{
	}

	void AlicaRosCommunication::sendSyncReady(SyncReady sr)
	{
	}

	void AlicaRosCommunication::sendSyncTalk(SyncTalk st)
	{
	}

} /* namespace alicaRosProxy */
