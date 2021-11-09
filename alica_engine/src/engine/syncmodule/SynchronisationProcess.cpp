#include "engine/syncmodule/SynchronisationProcess.h"
#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/TeamObserver.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/model/Synchronisation.h"
#include "engine/model/Transition.h"
#include "engine/syncmodule/SyncModule.h"
#include "engine/syncmodule/SyncRow.h"

//#define ALICA_DEBUG_LEVEL_DEBUG
#include <alica_common_config/debug_output.h>

namespace alica
{

using std::mutex;
using std::shared_ptr;

SynchronisationProcess::SynchronisationProcess(const AlicaEngine* ae, uint64_t myID, const Synchronisation* sync, SyncModule* sm)
        : _ae(ae)
        , _myID(myID)
        , _synchronisation(sync)
        , _syncModule(sm)
        , _readyForSync(false)
        , _lastTick(0)
        , _runningPlan(nullptr)
        , _myRow(nullptr)
        , _lastTalkData(nullptr)
        , _synchronisationDone(false)
{
    _syncStartTime = _ae->getAlicaClock().now();
    for (const Transition* t : sync->getInSync()) {
        _connectedTransitions.push_back(t->getId());
    }
}

SynchronisationProcess::~SynchronisationProcess()
{
    std::lock_guard<mutex> lock(_syncMutex);
    for (auto row : _syncMatrix) {
        delete row;
    }
}

const Synchronisation* SynchronisationProcess::getSynchronisation() const
{
    return _synchronisation;
}

void SynchronisationProcess::setTick(uint64_t now)
{
    _lastTick = now;
}

void SynchronisationProcess::changeOwnData(int64_t transitionID, bool conditionHolds)
{
    if (!conditionHolds) {
        // my condition does not hold => not ready for syncing
        std::lock_guard<mutex> lock(_syncMutex);
        _readyForSync = false;
    }

    SyncData sd;
    sd.agentID = _myID;
    sd.transitionID = transitionID;
    sd.conditionHolds = conditionHolds;
    sd.ack = false;

    bool maySendTalk = true;

    {
        std::lock_guard<mutex> lock(_syncMutex);
        if (_myRow) {
            if (sd.conditionHolds != _myRow->getSyncData().conditionHolds || *(sd.agentID) != *(_myRow->getSyncData().agentID) ||
                    sd.transitionID != _myRow->getSyncData().transitionID) {
                // my sync row has changed
                _myRow->setSyncData(sd);
                _myRow->editReceivedBy().clear();
                _myRow->editReceivedBy().push_back(_myID);
                _readyForSync = false;
            } else {
                maySendTalk = false;
            }
        } else {
            // init my row
            SyncRow* sr = new SyncRow(sd);
            if (find(sr->getReceivedBy().begin(), sr->getReceivedBy().end(), _myID) == sr->getReceivedBy().end()) {
                sr->editReceivedBy().push_back(_myID);
            }
            _myRow = sr;
            _syncMatrix.push_back(sr);
        }
    }

    if (maySendTalk) {
        std::lock_guard<mutex> lock(_syncMutex);
        if (isSyncComplete()) {
            ALICA_DEBUG_MSG("[SP (" << _myID << ")]: ChangedOwnData: Synchronisation " << _synchronisation->getId() << " ready");
            sendSyncReady();
            _readyForSync = true;
        } else {
            sendTalk(sd);
        }
    }
}

/**
 * resend lastTalk, test for failure on synchronisation timeout and stop sync on state change
 */
bool SynchronisationProcess::isValid(uint64_t curTick)
{
    bool stillActive = (_lastTick + 2 >= curTick);

    if (!stillActive) {
        // notify others if I am part of the synchronisation already (i.e. have an own row)
        if (_myRow != nullptr) {
            _myRow->editSyncData().conditionHolds = false;
            ALICA_DEBUG_MSG("[SP (" << _myID << ")]: isValid(): Failed due to Sync Timeout!");
            sendTalk(_myRow->getSyncData());
        }
        ALICA_DEBUG_MSG("[SP (" << _myID << ")]: isValid(): Not active!!");
        return false;
    }

    AlicaTime now = _ae->getAlicaClock().now();

    if (_lastTalkTime != AlicaTime::zero()) // talked already
    {
        if ((now > _synchronisation->getTalkTimeOut() + _lastTalkTime) && !_readyForSync) {
            if (_myRow != nullptr) {
                sendTalk(_myRow->getSyncData());
            }
        }
    }

    if (_synchronisation->isFailOnSyncTimeOut()) {
        if (now > _synchronisation->getSyncTimeOut() + _syncStartTime) {
            ALICA_DEBUG_MSG("[SP (" << _myID << ")]: Failed due to Sync Timeout!");
            return false;
        }
    }

    return true;
}

bool SynchronisationProcess::integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick)
{
    if (_readyForSync) {
        // do not integrate talks if we believe the sync is already finished
        return true;
    }

    bool isSynching = (_lastTick + 2 >= curTick);
    if (!isSynching) {
        // do not accept messages (and send unnecessary ACKs) if we are not in a state for sync
        return false;
    }

    ALICA_DEBUG_MSG("[SP (" << _myID << ")]: Integrate SyncTalk in synchronisation");

    for (const SyncData& sd : talk->syncData) {
        std::lock_guard<mutex> lock(_syncMutex);
        {
            // find row matching the received data
            SyncRow* rowInMatrix = nullptr;
            for (SyncRow* row : _syncMatrix) {
                if (sd.conditionHolds == row->getSyncData().conditionHolds && *(sd.agentID) == *(row->getSyncData().agentID) &&
                        sd.transitionID == row->getSyncData().transitionID) {
                    rowInMatrix = row;
                    break;
                }
            }

            if (!rowInMatrix) {
                SyncRow* newRow = new SyncRow(sd);
                newRow->editReceivedBy().push_back(talk->senderID);
                _syncMatrix.push_back(newRow);
            } else {
                if (find(rowInMatrix->getReceivedBy().begin(), rowInMatrix->getReceivedBy().end(), talk->senderID) == rowInMatrix->getReceivedBy().end()) {
                    rowInMatrix->editReceivedBy().push_back(talk->senderID);
                }
            }

            if (isSyncComplete()) {
                ALICA_DEBUG_MSG("[SP (" << _myID << ")]: Synchronisation " << _synchronisation->getId() << " ready");
                sendSyncReady();
                _readyForSync = true;
            } else {
                ALICA_DEBUG_MSG("[SP (" << _myID << ")]: Synchronisation " << _synchronisation->getId() << " not ready");
                // always reset this in case someone revokes his commitment
                _readyForSync = false;
            }

            // late acks...
            if (_readyForSync) {
                if (allSyncReady()) {
                    ALICA_DEBUG_MSG(
                            "[SP (" << _myID << ")]: Synchronisation successful (IntTalk) - elapsed time: " << (_ae->getAlicaClock().now() - _syncStartTime));
                    // notify sync module
                    _syncModule->synchronisationDone(_synchronisation);
                    _synchronisationDone = true;
                }
            }
        }
    }
    ALICA_DEBUG_MSG("[SP (" << _myID << ")]: Process after integrating SyncTalk: " << std::endl << *this);
    return true;
}

void SynchronisationProcess::integrateSyncReady(shared_ptr<SyncReady> ready)
{
    // every robot that has acknowledged my row needs to send me a SyncReady
    bool found = false;
    for (std::shared_ptr<SyncReady>& sr : _receivedSyncReadys) {
        if (*(sr->senderID) == *(ready->senderID)) {
            found = true;
            break;
        }
    }

    if (!found) {
        _receivedSyncReadys.push_back(ready);
    }

    ALICA_DEBUG_MSG("[SP (" << _myID << ")]: Process after integrating SyncReady: " << std::endl << *this);
    // check if all robots are ready
    if (_readyForSync) {
        if (allSyncReady()) {
            // notify _syncModul
            ALICA_DEBUG_MSG("[SP (" << _myID << ")]: Synchronisation successful (IntReady) - elapsed time: " << (_ae->getAlicaClock().now() - _syncStartTime));
            _syncModule->synchronisationDone(_synchronisation);
            _synchronisationDone = true;
        }
    }
}

void SynchronisationProcess::sendTalk(const SyncData& sd)
{
    SyncTalk talk;
    talk.syncData.push_back(sd);
    _lastTalkTime = _ae->getAlicaClock().now();
    _syncModule->sendSyncTalk(talk);
}

void SynchronisationProcess::sendSyncReady()
{
    // send own row again to be sure
    sendTalk(_myRow->getSyncData());

    SyncReady sr;
    sr.synchronisationID = _synchronisation->getId();
    _syncModule->sendSyncReady(sr);
}

/**
 * Test if all agents who acknowledged _myRow have sent a SyncReady
 * @return
 */
bool SynchronisationProcess::allSyncReady() const
{
    for (uint64_t robotID : _myRow->getReceivedBy()) {
        // we do not necessarily need an ack from ourselves
        if (robotID == _myID) {
            continue;
        }

        bool foundRobot = false;
        for (const std::shared_ptr<SyncReady>& sr : _receivedSyncReadys) {
            if (sr->senderID == robotID) {
                foundRobot = true;
                break;
            }
        }

        // at least one robot is missing
        if (!foundRobot) {
            return false;
        }
    }
    return true;
}

/**
 * TODO: drop this requirement by using a reentrent lock in this method
 * Before calling this method lock this mutex --> _syncMutex (lock_guard<mutex> lock(_syncMutex))
 */
bool SynchronisationProcess::isSyncComplete()
{
    // _myRow needs to be acknowledged by all participants
    // every participant needs to believe in its condition
    // there must be at least one participant for every condition
    std::lock_guard<std::mutex> lock(_rowOkMutex);
    _rowsOK.clear();
    // collect participants
    for (int64_t transID : _connectedTransitions) {
        SyncRow* foundRow = nullptr;

        for (SyncRow* row : _syncMatrix) {
            if (row->getSyncData().transitionID == transID && row->getSyncData().conditionHolds) {
                foundRow = row;
                break;
            }
        }
        if (!foundRow) {
            // no agent for transition
            return false;
        }

        if (find(_rowsOK.begin(), _rowsOK.end(), foundRow) == _rowsOK.end()) {
            _rowsOK.push_back(foundRow);
        }
    }

    // check for ACKs in own row
    if (!_myRow) {
        return false;
    }
    for (SyncRow* row : _rowsOK) {
        uint64_t agentID = row->getSyncData().agentID;
        if (std::find(_myRow->getReceivedBy().begin(), _myRow->getReceivedBy().end(), agentID) == _myRow->getReceivedBy().end()) {
            return false;
        }
    }
    return true;
}

std::ostream& operator<<(std::ostream& s, const SynchronisationProcess& syncProc)
{
    s << "Matrix:" << std::endl;
    for (SyncRow* row : syncProc._syncMatrix) {
        s << *row;
    }

    s << "ReceivedSyncReadys: ";
    for (const shared_ptr<SyncReady>& sr : syncProc._receivedSyncReadys) {
        s << sr->senderID << ", ";
    }
    s << std::endl;
    return s;
}

} // namespace alica
/* namespace alica */
