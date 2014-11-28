/*
 * IAlicaCommunication.cpp
 *
 *  Created on: 09.09.2014
 *      Author: endy
 */

#include "engine/IAlicaCommunication.h"
#include "engine/syncmodul/SyncModul.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/teamobserver/TeamObserver.h"
#include "engine/constraintmodul/IVariableSyncModule.h"

#include <iostream>

using namespace alica;

IAlicaCommunication::IAlicaCommunication(AlicaEngine* ae)
{
	this->ae = ae;
}

void alica::IAlicaCommunication::onSyncTalkReceived(shared_ptr<SyncTalk> st)
{
	ae->getSyncModul()->onSyncTalk(st);
}

void alica::IAlicaCommunication::onSyncReadyReceived(shared_ptr<SyncReady> sr)
{
	ae->getSyncModul()->onSyncReady(sr);
}

void alica::IAlicaCommunication::onAuthorityInfoReceived(shared_ptr<AllocationAuthorityInfo> aai)
{
	ae->getAuth()->handleIncomingAuthorityMessage(aai);
}

void alica::IAlicaCommunication::onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti)
{
	ae->getTeamObserver()->handlePlanTreeInfo(pti);
}

void alica::IAlicaCommunication::onSolverResult(shared_ptr<SolverResult> sr)
{
	ae->getResultStore()->onSolverResult(sr);
}
