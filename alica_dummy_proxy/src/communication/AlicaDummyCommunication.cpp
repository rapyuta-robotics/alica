/*
 * AlicaDummyCommunication.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Paul Panin
 */

#include "communication/AlicaDummyCommunication.h"

namespace alica_dummy_proxy
{

AlicaDummyCommunication::AlicaDummyCommunication(alica::AlicaEngine* ae)
    : alica::IAlicaCommunication(ae)
{
}

AlicaDummyCommunication::~AlicaDummyCommunication() {}
void AlicaDummyCommunication::sendAllocationAuthority(const alica::AllocationAuthorityInfo& /*aai*/) const {}
void AlicaDummyCommunication::sendAlicaEngineInfo(const alica::AlicaEngineInfo& /*ai*/) const {}
void AlicaDummyCommunication::sendPlanTreeInfo(const alica::PlanTreeInfo& /*pti*/) const {}
void AlicaDummyCommunication::sendRoleSwitch(const alica::RoleSwitch& /*rs*/) const {}
void AlicaDummyCommunication::sendSyncReady(const alica::SyncReady& /*sr*/) const {}
void AlicaDummyCommunication::sendSyncTalk(const alica::SyncTalk& /*st*/) const {}
void AlicaDummyCommunication::sendSolverResult(const alica::SolverResult& /*sr*/) const {}

void AlicaDummyCommunication::tick() {}
void AlicaDummyCommunication::startCommunication() {}
void AlicaDummyCommunication::stopCommunication() {}

} // namespace alica_dummy_proxy
