#pragma once
//#define SM_MISC
//#define SM_SUCCESS
//#define SM_FAILURE
//#define SM_MESSAGES

#include "engine/AgentIDConstPtr.h"
#include "engine/AlicaClock.h"
#include "engine/Types.h"

#include <memory>
#include <mutex>
#include <ostream>

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
    Synchronisation(AlicaEngine* ae, AgentIDConstPtr id, const SyncTransition* st, SyncModule* sm);
    virtual ~Synchronisation();
    void setTick(uint64_t now);
    void changeOwnData(int64_t transitionID, bool conditionHolds);
    bool isValid(uint64_t curTick);
    bool integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick);
    void integrateSyncReady(std::shared_ptr<SyncReady> ready);
    const SyncTransition* getSyncTransition() const;
    void setSyncTransition(const SyncTransition* syncTransition);

private:
    bool allSyncReady() const;
    friend std::ostream& operator<<(std::ostream& s, const Synchronisation& sync);

protected:
    AlicaEngine* _ae;
    std::mutex _syncMutex;
    std::mutex _rowOkMutex;
    SyncModule* _syncModul;
    const SyncTransition* _syncTransition;
    AgentIDConstPtr _myID;
    SyncData* _lastTalkData;
    AlicaTime _lastTalkTime;
    AlicaTime _syncStartTime;
    bool _readyForSync;
    uint64_t _lastTick;
    std::vector<std::shared_ptr<SyncReady>> _receivedSyncReadys;
    std::vector<int64_t> _connectedTransitions;
    RunningPlan* _runningPlan;
    std::vector<SyncRow*> _rowsOK;
    std::vector<SyncRow*> _syncMatrix;
    SyncRow* _myRow;

    void sendTalk(const SyncData& sd);
    void sendSyncReady();
    bool isSyncComplete();
};
std::ostream& operator<<(std::ostream& s, const Synchronisation& sync);
} /* namespace alica */
