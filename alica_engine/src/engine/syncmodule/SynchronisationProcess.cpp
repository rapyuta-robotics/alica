#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/TeamObserver.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/model/Synchronisation.h"
#include "engine/model/Transition.h"
#include <engine/syncmodule/SyncModule.h>
#include <engine/syncmodule/SyncRow.h>
#include <engine/syncmodule/SynchronisationProcess.h>

#include <alica_common_config/debug_output.h>
namespace alica
{

using std::mutex;
using std::shared_ptr;

SynchronisationProcess::SynchronisationProcess(const AlicaEngine* ae)
        : _myID(nullptr)
        , _ae(ae)
        , _syncModule(nullptr)
        , _synchronisation(nullptr)
        , _readyForSync(false)
        , _lastTick(0)
        , _runningPlan(nullptr)
        , _myRow(nullptr)
        , _lastTalkData(nullptr)
{
}

SynchronisationProcess::SynchronisationProcess(const AlicaEngine* ae, essentials::IdentifierConstPtr myID, const Synchronisation* sync, SyncModule* sm)
        : _ae(ae)
        , _myID(myID)
        , _synchronisation(sync)
        , _syncModule(sm)
        , _readyForSync(false)
        , _lastTick(0)
        , _runningPlan(nullptr)
        , _myRow(nullptr)
        , _lastTalkData(nullptr)
{
    _syncStartTime = _ae->getAlicaClock().now();
    for (const Transition* t : sync->getInSync()) {
        _connectedTransitions.push_back(t->getId());
    }
}

SynchronisationProcess::~SynchronisationProcess()
{
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
#ifdef SM_MISC
    std::cout << "SP: ChangeOwnData: ElapsedTime: " << (_ae->getAlicaClock().now() - _syncStartTime) << std::endl;
#endif

    if (!conditionHolds) {
        // my condition does not hold => not ready for syncing
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
        if (_myRow != nullptr) {
            if (/*sd->ack != _myRow->getSyncData()->ack
                                                ||*/
                    sd.conditionHolds != _myRow->getSyncData().conditionHolds || *(sd.agentID) != *(_myRow->getSyncData().agentID) ||
                    sd.transitionID != _myRow->getSyncData().transitionID) {
                // my sync row has changed
                _myRow->setSyncData(sd);
                _myRow->getReceivedBy().clear();
                _readyForSync = false;
                _myRow->getReceivedBy().push_back(_myID);
            } else {
#ifdef SM_MISC
                std::cout << "SP: ChangeOwnData: SendTalk==false" << std::endl;
#endif
                maySendTalk = false;
            }
        } else {
            // init my row
            SyncRow* sr = new SyncRow(sd);
            sr->getReceivedBy().push_back(_myID);
            _myRow = sr;
            _syncMatrix.push_back(sr);
        }
    }

#ifdef SM_MISC
    std::cout << std::endl;
    std::cout << "SP: ChangeOwnData:";
    std::cout << *this << std::endl;
#endif

    if (maySendTalk) {
        std::lock_guard<mutex> lock(_syncMutex);
        if (isSyncComplete()) {
#ifdef SM_SUCCESS
            std::cout << "SP: ChangedOwnData: Synchronisation " << _synchronisation->getId() << " ready" << std::endl;
#endif
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
            if (_myRow->hasData()) {
                _myRow->editSyncData().conditionHolds = false;
                ALICA_DEBUG_MSG("SP: isValid(): Failed due to Sync Timeout!");
                sendTalk(_myRow->getSyncData());
            }
        }
        ALICA_DEBUG_MSG("SP: isValid(): Not active!!");
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

    ALICA_DEBUG_MSG("SP: isValid(): syncStartTime " << _syncStartTime);

    if (_synchronisation->isFailOnSyncTimeOut()) {
        if (now > _synchronisation->getSyncTimeOut() + _syncStartTime) {
            ALICA_DEBUG_MSG("SP: Failed due to Sync Timeout!");
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
        // do not accept messages (and send uneccessary ACKs) if we are not in a state for sync
        return false;
    }

    ALICA_DEBUG_MSG("Integrate synctalk in synchronisation");
    ALICA_DEBUG_MSG("ST: ElapsedTime: " << (_ae->getAlicaClock().now() - _syncStartTime));

    for (const SyncData& sd : talk->syncData) {
        ALICA_DEBUG_MSG("syncdata for transID: " << sd.transitionID);

        std::lock_guard<mutex> lock(_syncMutex);
        {
            SyncRow* rowInMatrix = nullptr;
            for (SyncRow* row : _syncMatrix) {
                ALICA_DEBUG_MSG("ROW SD: " << row->getSyncData().agentID << " " << row->getSyncData().transitionID << " " << row->getSyncData().conditionHolds
                                           << " " << row->getSyncData().ack);
                ALICA_DEBUG_MSG("CUR SD: " << sd.agentID << " " << sd.transitionID << " " << sd.conditionHolds << " " << sd.ack);

                if (/*sd.ack == row->getSyncData()->ack
                                                        &&*/
                        sd.conditionHolds == row->getSyncData().conditionHolds && *(sd.agentID) == *(row->getSyncData().agentID) &&
                        sd.transitionID == row->getSyncData().transitionID) {
                    rowInMatrix = row;
                    break;
                }
            }

            // if(rowInMatrix != null)
            if (rowInMatrix == nullptr) {
                ALICA_DEBUG_MSG("NEW MATRIX row");
                SyncRow* newRow = new SyncRow(sd);
                newRow->getReceivedBy().push_back(talk->senderID);
                _syncMatrix.push_back(newRow);
            } else {
                ALICA_DEBUG_MSG("Received by: " << talk->senderID);
                rowInMatrix->getReceivedBy().push_back(talk->senderID);
            }
            if (isSyncComplete()) {
                ALICA_DEBUG_MSG("IntegrateSyncTalk: Synchronisation " << _synchronisation->getId() << " ready");

                sendSyncReady();
                _readyForSync = true;
            } else {
                // always reset this in case someone revokes his commitment
                _readyForSync = false;
            }

            ALICA_DEBUG_MSG("Matrix: IntSyncTalk");
            ALICA_DEBUG_MSG(*this);

            // late acks...
            if (_readyForSync) {
                if (allSyncReady()) {
                    ALICA_DEBUG_MSG("SyncDONE in Synchronisation (IntTalk): elapsed time: " << (_ae->getAlicaClock().now() - _syncStartTime));
                    // notify syncmodule
                    _syncModule->synchronisationDone(_synchronisation);
                }
            }
        }
    }

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
    ALICA_DEBUG_MSG("Matrix: IntSyncReady");
    ALICA_DEBUG_MSG(*this);

    // check if all robots are ready
    if (_readyForSync) {
        if (allSyncReady()) {
            // notify _syncModul
            ALICA_DEBUG_MSG("SyncDONE in Synchronisation (IntReady): elapsed time: " << (_ae->getAlicaClock().now() - _syncStartTime));
            _syncModule->synchronisationDone(_synchronisation);
        }
    }
}

void SynchronisationProcess::setSynchronisation(const Synchronisation* synchronisation)
{
    _synchronisation = synchronisation;
}

bool SynchronisationProcess::allSyncReady() const
{
    // test if all robots who acknowledged _myRow have sent a SyncReady
    for (essentials::IdentifierConstPtr robotID : _myRow->getReceivedBy()) {
        if (robotID != _myID) // we do not necessarily need an ack from ourselves
        {
            bool foundRobot = false;
            for (const std::shared_ptr<SyncReady>& sr : _receivedSyncReadys) {
                if (sr->senderID == robotID) {
                    foundRobot = true;
                    break;
                }
            }

            if (!foundRobot) // at least one robot is missing
            {
                return false;
            }
        }
    }
    return true;
}

void SynchronisationProcess::sendTalk(const SyncData& sd)
{
    SyncTalk talk;
    talk.syncData.push_back(sd);
    _lastTalkTime = _ae->getAlicaClock().now();

    ALICA_DEBUG_MSG("SP: Sending Talk TID: " << sd.transitionID);

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
            // no robot for transition
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
        essentials::IdentifierConstPtr tmp = row->getSyncData().agentID;
        if (std::find(_myRow->getReceivedBy().begin(), _myRow->getReceivedBy().end(), tmp) == _myRow->getReceivedBy().end()) {
            return false;
        }
    }
    return true;
}

std::ostream& operator<<(std::ostream& s, const SynchronisationProcess& syncProc)
{
    s << std::endl;
    s << "Matrix:" << std::endl;

    for (SyncRow* row : syncProc._syncMatrix) {
        s << "Row: " << row->getSyncData().agentID << " "
          << std::to_string(row->getSyncData().transitionID) + " " + (row->getSyncData().conditionHolds ? " COND" : "!COND") << " "
          << (row->getSyncData().ack ? " ACK" : "!ACK") << " RecvBy: ";
        for (essentials::IdentifierConstPtr robotID : row->getReceivedBy()) {
            s << robotID << ", ";
        }
        s << std::endl;
    }
    s << "ReceivedSyncReadys: ";
    for (const shared_ptr<SyncReady>& sr : syncProc._receivedSyncReadys) {
        s << sr->senderID << ", " << std::endl;
    }
    s << std::endl;
    return s;
}

} // namespace alica
/* namespace alica */
