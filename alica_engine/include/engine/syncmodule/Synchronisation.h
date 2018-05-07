#pragma once
//#define SM_MISC
//#define SM_SUCCESS
//#define SM_FAILURE
//#define SM_MESSAGES

#include "engine/AlicaClock.h"
#include "engine/Types.h"
#include "supplementary/AgentID.h"

#include <memory>
#include <mutex>

namespace alica
{

class SyncModule;
class SyncTransition;
class SyncRow;
class RunningPlan;
struct SyncData;
struct SyncReady;
struct SyncTalk;
class AlicaEngine;

class Synchronisation
{
  public:
    Synchronisation(AlicaEngine* ae);
    Synchronisation(AlicaEngine* ae, const supplementary::AgentID* myID, const SyncTransition* st, SyncModule* sm);
    virtual ~Synchronisation();
    void setTick(uint64_t now);
    void changeOwnData(int64_t transitionID, bool conditionHolds);
    bool isValid(uint64_t curTick);
    bool integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick);
    void integrateSyncReady(std::shared_ptr<SyncReady> ready);
    const SyncTransition* getSyncTransition() const;
    void setSyncTransition(const SyncTransition* syncTransition);

  private:
    bool allSyncReady();
    void printMatrix();

  protected:
    AlicaEngine* ae;
    std::mutex syncMutex;
    std::mutex rowOkMutex;
    SyncModule* syncModul;
    const SyncTransition* syncTransition;
    AgentIDConstPtr myID;
    AlicaTime lastTalkTime;
    SyncData* lastTalkData;
    AlicaTime syncStartTime;
    bool readyForSync;
    uint64_t lastTick;
    std::vector<std::shared_ptr<SyncReady>> receivedSyncReadys;
    std::vector<int64_t> connectedTransitions;
    RunningPlan* runningPlan;
    std::vector<SyncRow*> rowsOK;
    std::vector<SyncRow*> syncMatrix;
    SyncRow* myRow;

    void sendTalk(SyncData* sd);
    void sendSyncReady();
    bool isSyncComplete();
};

} /* namespace alica */
