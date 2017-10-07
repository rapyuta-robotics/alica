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
	void AlicaDummyCommunication::sendAllocationAuthority(alica::AllocationAuthorityInfo& aai) const
	{

	}
	void AlicaDummyCommunication::sendAlicaEngineInfo(alica::AlicaEngineInfo& ai) const
	{

	}
	void AlicaDummyCommunication::sendPlanTreeInfo(alica::PlanTreeInfo& pti) const
	{

	}
	void AlicaDummyCommunication::sendRoleSwitch(alica::RoleSwitch& rs) const
	{

	}
	void AlicaDummyCommunication::sendSyncReady(alica::SyncReady& sr) const
	{

	}
	void AlicaDummyCommunication::sendSyncTalk(alica::SyncTalk& st) const
	{

	}
	void AlicaDummyCommunication::sendSolverResult(alica::SolverResult& sr) const
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
