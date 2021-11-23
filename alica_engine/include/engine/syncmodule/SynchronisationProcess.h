#pragma once

#include "engine/AlicaClock.h"
#include "engine/Types.h"

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
    SynchronisationProcess(const AlicaEngine* ae, AgentId myID, const Synchronisation* sync, SyncModule* sm);
    virtual ~SynchronisationProcess();
    void setTick(uint64_t now);
    void changeOwnData(int64_t transitionID, bool conditionHolds);
    bool isValid(uint64_t curTick);
    bool isSynchronisationDone() { return _synchronisationDone; }
    bool integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick);
    void integrateSyncReady(std::shared_ptr<SyncReady> ready);
    const Synchronisation* getSynchronisation() const;

private:
    bool allSyncReady() const;
    void sendTalk(const SyncData& sd);
    void sendSyncReady();
    bool isSyncComplete();

    friend std::ostream& operator<<(std::ostream& s, const SynchronisationProcess& sync);

    const AlicaEngine* _ae;
    std::mutex _syncMutex;
    std::mutex _rowOkMutex;
    SyncModule* _syncModule;
    const Synchronisation* _synchronisation;
    AgentId _myID;
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
    bool _synchronisationDone;
};
std::ostream& operator<<(std::ostream& s, const SynchronisationProcess& syncProc);
} /* namespace alica */
