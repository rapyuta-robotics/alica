#pragma once
//#define SM_MISC
//#define SM_SUCCESS
//#define SM_FAILURE
//#define SM_MESSAGES

#include "engine/AlicaClock.h"
#include "engine/Types.h"

#include <essentials/AgentIDConstPtr.h>

#include <memory>
#include <mutex>
#include <ostream>

namespace alica
{

class SyncModule;
class Synchronisation;
class SyncRow;
class RunningPlan;
struct SyncData;
struct SyncReady;
struct SyncTalk;
class AlicaEngine;

class SynchronisationProcess
{
public:
    SynchronisationProcess(AlicaEngine* ae);
    SynchronisationProcess(AlicaEngine* ae, essentials::AgentIDConstPtr id, const Synchronisation* sync, SyncModule* sm);
    virtual ~SynchronisationProcess();
    void setTick(uint64_t now);
    void changeOwnData(int64_t transitionID, bool conditionHolds);
    bool isValid(uint64_t curTick);
    bool integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick);
    void integrateSyncReady(std::shared_ptr<SyncReady> ready);
    const Synchronisation* getSynchronisation() const;
    void setSynchronisation(const Synchronisation *synchronisation);

private:
    bool allSyncReady() const;
    friend std::ostream& operator<<(std::ostream& s, const SynchronisationProcess& sync);

protected:
    AlicaEngine* ae;
    std::mutex syncMutex;
    std::mutex rowOkMutex;
    SyncModule* syncModul;
    const Synchronisation* synchronisation;
    essentials::AgentIDConstPtr myID;
    SyncData* lastTalkData;
    AlicaTime lastTalkTime;
    AlicaTime syncStartTime;
    bool readyForSync;
    uint64_t lastTick;
    std::vector<std::shared_ptr<SyncReady>> receivedSyncReadys;
    std::vector<int64_t> connectedTransitions;
    RunningPlan* runningPlan;
    std::vector<SyncRow*> rowsOK;
    std::vector<SyncRow*> syncMatrix;
    SyncRow* myRow;

    void sendTalk(const SyncData& sd);
    void sendSyncReady();
    bool isSyncComplete();
};
std::ostream& operator<<(std::ostream& s, const SynchronisationProcess& syncProc);
} /* namespace alica */
