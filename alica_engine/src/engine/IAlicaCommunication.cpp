#include "engine/IAlicaCommunication.h"
#include <engine/syncmodule/SyncModule.h>

#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/TeamObserver.h"
#include "engine/allocationauthority/AuthorityManager.h"
#include "engine/constraintmodul/VariableSyncModule.h"
#include "engine/containers/PlanTreeInfo.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"

#include <iostream>

using std::shared_ptr;

void alica::IAlicaCommunication::onSyncTalkReceived(shared_ptr<SyncTalk> st)
{
    if (!ae) {
        // case for testing without engine
        std::cout << "\033[93mRecieving ST: " << st->senderID << ' ' << st->syncData.size() << "\033[0m\n";
    } else {
        ae->getSyncModul()->onSyncTalk(st);
    }
}

void alica::IAlicaCommunication::onSyncReadyReceived(shared_ptr<SyncReady> sr)
{
    if (!ae) {
        // case for testing without engine
        std::cout << "\033[93mRecieving SR: " << sr->senderID << ' ' << sr->synchronisationID << "\033[0m\n";
    } else {
        ae->getSyncModul()->onSyncReady(sr);
    }
}

void alica::IAlicaCommunication::onAuthorityInfoReceived(const AllocationAuthorityInfo& aai)
{
    if (!ae) {
        // case for testing without engine
         std::cout << "\033[93mRecieving AAI: " << aai.senderID << ' ' << aai.authority << ' ' << aai.parentState <<
                                     ' ' << aai.planType << ' ' << aai.planId <<"\033[0m\n";
    } else {
        ae->getAuth()->handleIncomingAuthorityMessage(aai);
    }
}

void alica::IAlicaCommunication::onPlanTreeInfoReceived(shared_ptr<PlanTreeInfo> pti)
{
    if (!ae) {
        // case for testing without engine
        std::cout << "\033[93mRecieving PTI: " << pti->senderID << ' ' << pti->succeededEPs.size() << ' ' << pti->stateIDs.size() << "\033[0m\n";
    } else {
        ae->getTeamObserver()->handlePlanTreeInfo(pti);
    }
}

void alica::IAlicaCommunication::onSolverResult(const SolverResult& sr)
{
    if (!ae) {
        // case for testing without engine
        std::cout << "\033[93mRecieving SR: " << sr.senderID << ' ' << sr.vars.size() << "\033[0m\n";
    } else {
        ae->getResultStore()->onSolverResult(sr);
    }
}
