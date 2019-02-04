#include "engine/AlicaClock.h"
#include "engine/AlicaEngine.h"
#include "engine/TeamObserver.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/model/SyncTransition.h"
#include "engine/model/Transition.h"
#include <engine/syncmodule/SyncModule.h>
#include <engine/syncmodule/SyncRow.h>
#include <engine/syncmodule/Synchronisation.h>

#include <alica_common_config/debug_output.h>

namespace alica
{

using std::mutex;
using std::shared_ptr;

Synchronisation::Synchronisation(const AlicaEngine* ae)
        : _myID(nullptr)
        , _ae(ae)
        , _syncModul(nullptr)
        , _syncTransition(nullptr)
        , _readyForSync(false)
        , _lastTick(0)
        , _runningPlan(nullptr)
        , _myRow(nullptr)
        , _lastTalkData(nullptr)
{
}

Synchronisation::Synchronisation(const AlicaEngine* ae, AgentIDConstPtr myID, const SyncTransition* st, SyncModule* sm)
        : _ae(ae)
        , _myID(myID)
        , _syncTransition(st)
        , _syncModul(sm)
        , _readyForSync(false)
        , _lastTick(0)
        , _runningPlan(nullptr)
        , _myRow(nullptr)
        , _lastTalkData(nullptr)
{
    _syncStartTime = _ae->getAlicaClock().now();
    for (const Transition* t : st->getInSync()) {
        _connectedTransitions.push_back(t->getId());
    }
}

Synchronisation::~Synchronisation()
{
    for (auto row : _syncMatrix) {
        delete row;
    }
}

const SyncTransition* Synchronisation::getSyncTransition() const
{
    return _syncTransition;
}

void Synchronisation::setTick(uint64_t now)
{
    _lastTick = now;
}

void Synchronisation::changeOwnData(int64_t transitionID, bool conditionHolds)
{
#ifdef SM_MISC
    std::cout << "CHOD: ElapsedTime: " << (_ae->getAlicaClock().now() - _syncStartTime) << std::endl;
#endif

    if (!conditionHolds) {
        // my condition does not hold => not ready for syncing
        _readyForSync = false;
    }
    SyncData sd;
    sd.robotID = _myID;
    sd.transitionID = transitionID;
    sd.conditionHolds = conditionHolds;
    sd.ack = false;

    bool maySendTalk = true;

    {
        std::lock_guard<mutex> lock(_syncMutex);
        if (_myRow != nullptr) {
            if (/*sd->ack != _myRow->getSyncData()->ack
						||*/ sd.conditionHolds != _myRow->getSyncData().conditionHolds ||
                    *(sd.robotID) != *(_myRow->getSyncData().robotID) || sd.transitionID != _myRow->getSyncData().transitionID) {
                // my sync row has changed
                _myRow->setSyncData(sd);
                _myRow->getReceivedBy().clear();
                _readyForSync = false;
                _myRow->getReceivedBy().push_back(_myID);
            } else {
#ifdef SM_MISC
                cout << "ChangeOwnData: SendTalk==false" << endl;
#endif
                maySendTalk = false;
            }
        } else // init my row
        {
            SyncRow* sr = new SyncRow(sd);
            sr->getReceivedBy().push_back(_myID);
            _myRow = sr;

            _syncMatrix.push_back(sr);
        }
    }

#ifdef SM_MISC
    cout << endl;
    cout << "Matrix: ChangeOwnData" << endl;
    printMatrix();
#endif

    if (maySendTalk) {
        if (isSyncComplete()) {
#ifdef SM_SUCCESS
            cout << "ChangedOwnData: Synctrans " << _syncTransition->getId() << " ready" << endl;
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
bool Synchronisation::isValid(uint64_t curTick)
{
    bool stillActive = (_lastTick + 2 >= curTick);

    if (!stillActive) {
        // notify others if i am part of the synchronisation already (i.e. have an own row)
        if (_myRow != nullptr) {
            if (_myRow->hasData()) {
                _myRow->editSyncData().conditionHolds = false;

                sendTalk(_myRow->getSyncData());
            }
        }
        return false;
    }

    AlicaTime now = _ae->getAlicaClock().now();

    if (_lastTalkTime != AlicaTime::zero()) // talked already
    {
        ALICA_DEBUG_MSG("TestTimeOut on Sync: " << _syncTransition->getId());

        if ((now > _syncTransition->getTalkTimeOut() + _lastTalkTime) && !_readyForSync) {
            if (_myRow != nullptr) {
                sendTalk(_myRow->getSyncData());
            }
        }
    }

    ALICA_DEBUG_MSG("Synchronisation: TestTimeOut(): syncStarTime " << _syncStartTime);

    if (_syncTransition->isFailOnSyncTimeOut()) {
        if (now > _syncTransition->getSyncTimeOut() + _syncStartTime) {
            ALICA_DEBUG_MSG("Synchronisation: TestTimeOut() sync failed");
            return false;
        }
    }

    return true;
}

bool Synchronisation::integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick)
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
                ALICA_DEBUG_MSG("ROW SD: " << row->getSyncData().robotID << " " << row->getSyncData().transitionID << " " << row->getSyncData().conditionHolds
                                           << " " << row->getSyncData().ack);
                ALICA_DEBUG_MSG("CUR SD: " << sd.robotID << " " << sd.transitionID << " " << sd.conditionHolds << " " << sd.ack);

                if (/*sd.ack == row->getSyncData()->ack
							&&*/ sd.conditionHolds == row->getSyncData().conditionHolds &&
                        *(sd.robotID) == *(row->getSyncData().robotID) && sd.transitionID == row->getSyncData().transitionID) {
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
                ALICA_DEBUG_MSG("IntegrateSyncTalk: Synctrans " << _syncTransition->getId() << " ready");

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
                    // notify syncmodul
                    _syncModul->synchronisationDone(_syncTransition);
                }
            }
        }
    }

    return true;
}

void Synchronisation::integrateSyncReady(shared_ptr<SyncReady> ready)
{
    // every robot that has acknowleged my row needs to send me a SyncReady
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
            _syncModul->synchronisationDone(_syncTransition);
        }
    }
}

void Synchronisation::setSyncTransition(const SyncTransition* syncTransition)
{
    _syncTransition = syncTransition;
}

bool Synchronisation::allSyncReady() const
{
    // test if all robots who acknowledged _myRow have sent a SyncReady
    for (AgentIDConstPtr robotID : _myRow->getReceivedBy()) {
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

void Synchronisation::sendTalk(const SyncData& sd)
{
    SyncTalk talk;
    talk.syncData.push_back(sd);
    _lastTalkTime = _ae->getAlicaClock().now();

    ALICA_DEBUG_MSG("Sending Talk TID: " << sd.transitionID);

    _syncModul->sendSyncTalk(talk);
}

void Synchronisation::sendSyncReady()
{
    // send own row again to be sure
    sendTalk(_myRow->getSyncData());

    SyncReady sr;
    sr.syncTransitionID = _syncTransition->getId();
    _syncModul->sendSyncReady(sr);
}

/**
 * Before calling this method lock this mutex --> _syncMutex (lock_guard<mutex> lock(_syncMutex))
 */
bool Synchronisation::isSyncComplete()
{
    // _myRow needs to be acknowledged by all participants
    // every participant needs to believe in its condition
    // there must be at least one participant for every condition
    std::lock_guard<std::mutex> lock(_rowOkMutex);
    _rowsOK.clear();
    // collect participants
    for (int64_t transID : _connectedTransitions) {
        SyncRow* foundRow = nullptr;

        //			Is not needed here
        //			lock_guard<mutex> lock(_syncMutex);
        for (SyncRow* row : _syncMatrix) {
            if (row->getSyncData().transitionID == transID && row->getSyncData().conditionHolds) {
                foundRow = row;
                break;
            }
        }
        if (foundRow == nullptr) // no robot for transition
        {
            return false;
        }

        if (find(_rowsOK.begin(), _rowsOK.end(), foundRow) == _rowsOK.end()) {
            _rowsOK.push_back(foundRow);
        }
    }
    //		check for acks in own row
    if (!_myRow) {
        return false;
    }
    for (SyncRow* row : _rowsOK) {
        AgentIDConstPtr tmp = row->getSyncData().robotID;
        if (std::find(_myRow->getReceivedBy().begin(), _myRow->getReceivedBy().end(), tmp) == _myRow->getReceivedBy().end()) {
            return false;
        }
    }
    return true;
}

std::ostream& operator<<(std::ostream& s, const Synchronisation& sync)
{
    s << std::endl;
    s << "Matrix:" << std::endl;

    for (SyncRow* row : sync._syncMatrix) {
        s << "Row: " << row->getSyncData().robotID << " "
          << std::to_string(row->getSyncData().transitionID) + " " + std::to_string(row->getSyncData().conditionHolds) << " " << row->getSyncData().ack
          << " RecvBy: ";
        for (AgentIDConstPtr robotID : row->getReceivedBy()) {
            s << robotID << ", ";
        }
        s << std::endl;
    }
    s << "ReceivedSyncreadys: ";
    for (const shared_ptr<SyncReady>& sr : sync._receivedSyncReadys) {
        s << sr->senderID << ", " << std::endl;
        ;
    }
    s << std::endl;
    return s;
}

} // namespace alica
/* namespace alica */
