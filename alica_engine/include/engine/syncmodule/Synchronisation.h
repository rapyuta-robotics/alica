#pragma once
//#define SM_MISC
//#define SM_SUCCESS
//#define SM_FAILURE
//#define SM_MESSAGES

#include "engine/AlicaClock.h"
#include "supplementary/AgentID.h"
#include <list>
#include <mutex>
#include <memory>

namespace alica {
using std::list;
using std::mutex;
using std::shared_ptr;

class SyncModule;
class SyncTransition;
class SyncRow;
class RunningPlan;
struct SyncData;
struct SyncReady;
struct SyncTalk;
class AlicaEngine;

class Synchronisation {
public:
    Synchronisation(AlicaEngine* ae);
    Synchronisation(AlicaEngine* ae, const supplementary::AgentID* myID, SyncTransition* st, SyncModule* sm);
    virtual ~Synchronisation();
    void setTick(unsigned long now);
    void changeOwnData(long transitionID, bool conditionHolds);
    bool isValid(unsigned long curTick);
    bool integrateSyncTalk(shared_ptr<SyncTalk> talk, unsigned long curTick);
    void integrateSyncReady(shared_ptr<SyncReady> ready);
    SyncTransition* getSyncTransition();
    void setSyncTransition(SyncTransition* syncTransition);

private:
    bool allSyncReady();
    void printMatrix();

protected:
    AlicaEngine* ae;
    mutex syncMutex;
    mutex rowOkMutex;
    SyncModule* syncModul;
    SyncTransition* syncTransition;
    const supplementary::AgentID* myID;
    AlicaTime lastTalkTime;
    SyncData* lastTalkData;
    AlicaTime syncStartTime;
    bool readyForSync;
    unsigned long lastTick;
    list<shared_ptr<SyncReady>> receivedSyncReadys;
    list<long> connectedTransitions;
    RunningPlan* runningPlan;
    list<SyncRow*> rowsOK;
    SyncRow* myRow;
    list<SyncRow*> syncMatrix;
    void sendTalk(SyncData* sd);
    void sendSyncReady();
    bool isSyncComplete();
};

} /* namespace alica */
