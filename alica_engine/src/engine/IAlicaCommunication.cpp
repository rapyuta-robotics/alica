/*
 * IAlicaCommunication.cpp
 *
 *  Created on: 09.09.2014
 *      Author: endy
 */

#include "IAlicaCommunication.h"

using namespace alica;

IAlicaCommunication::IAlicaCommunication(AlicaEngine* ae)
{
	this->ae = ae;
}


void alica::IAlicaCommunication::onSyncTalkReceived(shared_ptr<SyncTalk> st)
{
	ae->syncModul->onSyncTalk(st);
}

void alica::IAlicaCommunication::onSyncReadyReceived(shared_ptr<SyncReady> sr)
{
	ae->syncModul->onSyncReady(sr);
}

void alica::IAlicaCommunication::onAuthorityInfoReceived(shared_ptr<AllocationAuthorityInfo> aai)
{
	ae->auth->handleIncomingAuthorityMessage(aai);
}

void alica::IAlicaCommunication::onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti)
{
	ae->teamObserver->handlePlanTreeInfo(pti);
}
