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

    SynchronisationProcess::SynchronisationProcess(AlicaEngine* ae)
        : myID(nullptr)
{
    this->ae = ae;
    this->syncModul = nullptr;
    this->synchronisation = nullptr;
    this->readyForSync = false;
    this->lastTick = 0;
    this->runningPlan = nullptr;
    this->myRow = nullptr;
    this->lastTalkData = nullptr;
}

    SynchronisationProcess::SynchronisationProcess(AlicaEngine* ae, essentials::IdentifierConstPtr myID, const Synchronisation* sync, SyncModule* sm)
{
    this->ae = ae;
    this->synchronisation = sync;
    this->myID = myID;
    this->syncStartTime = ae->getAlicaClock()->now();
    for (const Transition* t : sync->getInSync()) {
        connectedTransitions.push_back(t->getId());
    }
    this->syncModul = sm;
    this->runningPlan = nullptr;
    this->myRow = nullptr;
    this->readyForSync = false;
    this->lastTick = 0;
    this->lastTalkData = nullptr;
}

    SynchronisationProcess::~SynchronisationProcess()
{
    for (auto row : syncMatrix) {
        delete row;
    }
}

const Synchronisation* SynchronisationProcess::getSynchronisation() const
{
    return synchronisation;
}

void SynchronisationProcess::setTick(uint64_t now)
{
    this->lastTick = now;
}

void SynchronisationProcess::changeOwnData(int64_t transitionID, bool conditionHolds)
{
#ifdef SM_MISC
    std::cout << "CHOD: ElapsedTime: " << (ae->getAlicaClock()->now() - this->syncStartTime) << std::endl;
#endif

    if (!conditionHolds) {
        // my condition does not hold => not ready for syncing
        this->readyForSync = false;
    }
    SyncData sd;
    sd.robotID = this->myID;
    sd.transitionID = transitionID;
    sd.conditionHolds = conditionHolds;
    sd.ack = false;

    bool maySendTalk = true;

    {
        std::lock_guard<mutex> lock(syncMutex);
        if (myRow != nullptr) {
            if (/*sd->ack != myRow->getSyncData()->ack
                                                ||*/
                            sd.conditionHolds != myRow->getSyncData().conditionHolds ||
                    *(sd.robotID) != *(myRow->getSyncData().robotID) || sd.transitionID != myRow->getSyncData().transitionID) {
                // my sync row has changed
                myRow->setSyncData(sd);
                myRow->getReceivedBy().clear();
                this->readyForSync = false;
                myRow->getReceivedBy().push_back(this->myID);
            } else {
#ifdef SM_MISC
                cout << "ChangeOwnData: SendTalk==false" << endl;
#endif
                maySendTalk = false;
            }
        } else // init my row
        {
            SyncRow* sr = new SyncRow(sd);
            sr->getReceivedBy().push_back(this->myID);
            this->myRow = sr;

            this->syncMatrix.push_back(sr);
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
            cout << "ChangedOwnData: Synchronisation " << this->synchronisation->getId() << " ready" << endl;
#endif
            sendSyncReady();
            this->readyForSync = true;
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
    bool stillActive = (this->lastTick + 2 >= curTick);

    if (!stillActive) {
        // notify others if i am part of the synchronisation already (i.e. have an own row)
        if (myRow != nullptr) {
            if (myRow->hasData()) {
                myRow->editSyncData().conditionHolds = false;

                sendTalk(myRow->getSyncData());
            }
        }
        return false;
    }

    AlicaTime now = ae->getAlicaClock()->now();

    if (this->lastTalkTime != AlicaTime::zero()) // talked already
    {
        ALICA_DEBUG_MSG("TestTimeOut on Sync: " << this->synchronisation->getId());

        if ((now > this->synchronisation->getTalkTimeOut() + this->lastTalkTime) && !this->readyForSync) {
            if (this->myRow != nullptr) {
                sendTalk(this->myRow->getSyncData());
            }
        }
    }

    ALICA_DEBUG_MSG("Synchronisation: TestTimeOut(): syncStarTime " << this->syncStartTime);

    if (this->synchronisation->isFailOnSyncTimeOut()) {
        if (now > this->synchronisation->getSyncTimeOut() + this->syncStartTime) {
            ALICA_DEBUG_MSG("Synchronisation: TestTimeOut() sync failed");
            return false;
        }
    }

    return true;
}

bool SynchronisationProcess::integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick)
{
    if (this->readyForSync) {
        // do not integrate talks if we believe the sync is already finished
        return true;
    }

    bool isSynching = (this->lastTick + 2 >= curTick);

    if (!isSynching) {
        // do not accept messages (and send uneccessary ACKs) if we are not in a state for sync
        return false;
    }

    ALICA_DEBUG_MSG("Integrate synctalk in synchronisation");
    ALICA_DEBUG_MSG("ST: ElapsedTime: " << (ae->getAlicaClock()->now() - this->syncStartTime));

    for (const SyncData& sd : talk->syncData) {
        ALICA_DEBUG_MSG("syncdata for transID: " << sd.transitionID);

        std::lock_guard<mutex> lock(syncMutex);
        {
            SyncRow* rowInMatrix = nullptr;
            for (SyncRow* row : this->syncMatrix) {
                ALICA_DEBUG_MSG("ROW SD: " << row->getSyncData().robotID << " " << row->getSyncData().transitionID << " " << row->getSyncData().conditionHolds
                                           << " " << row->getSyncData().ack);
                ALICA_DEBUG_MSG("CUR SD: " << sd.robotID << " " << sd.transitionID << " " << sd.conditionHolds << " " << sd.ack);

                if (/*sd.ack == row->getSyncData()->ack
                                                        &&*/
                                sd.conditionHolds == row->getSyncData().conditionHolds &&
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
                syncMatrix.push_back(newRow);
            } else {
                ALICA_DEBUG_MSG("Received by: " << talk->senderID);
                rowInMatrix->getReceivedBy().push_back(talk->senderID);
            }
            if (isSyncComplete()) {
                ALICA_DEBUG_MSG("IntegrateSyncTalk: Synctrans " << this->synchronisation->getId() << " ready");

                sendSyncReady();
                this->readyForSync = true;
            } else {
                // always reset this in case someone revokes his commitment
                this->readyForSync = false;
            }

            ALICA_DEBUG_MSG("Matrix: IntSyncTalk");
            ALICA_DEBUG_MSG(*this);

            // late acks...
            if (this->readyForSync) {
                if (allSyncReady()) {
                    ALICA_DEBUG_MSG("SyncDONE in Synchronisation (IntTalk): elapsed time: " << (ae->getAlicaClock()->now() - syncStartTime));
                    // notify syncmodul
                    this->syncModul->synchronisationDone(this->synchronisation);
                }
            }
        }
    }

    return true;
}

void SynchronisationProcess::integrateSyncReady(shared_ptr<SyncReady> ready)
{
    // every robot that has acknowleged my row needs to send me a SyncReady
    bool found = false;
    for (std::shared_ptr<SyncReady>& sr : this->receivedSyncReadys) {
        if (*(sr->senderID) == *(ready->senderID)) {
            found = true;
            break;
        }
    }

    if (!found) {
        this->receivedSyncReadys.push_back(ready);
    }
    ALICA_DEBUG_MSG("Matrix: IntSyncReady");
    ALICA_DEBUG_MSG(*this);

    // check if all robots are ready
    if (this->readyForSync) {
        if (allSyncReady()) {
            // notify syncModul
            ALICA_DEBUG_MSG("SyncDONE in Synchronisation (IntReady): elapsed time: " << (ae->getAlicaClock()->now() - this->syncStartTime));
            this->syncModul->synchronisationDone(this->synchronisation);
        }
    }
}

void SynchronisationProcess::setSynchronisation(const Synchronisation *synchronisation)
{
    this->synchronisation = synchronisation;
}

bool SynchronisationProcess::allSyncReady() const
{
    // test if all robots who acknowledged myRow have sent a SyncReady
    for (essentials::IdentifierConstPtr robotID : this->myRow->getReceivedBy()) {
        if (robotID != myID) // we do not necessarily need an ack from ourselves
        {
            bool foundRobot = false;
            for (const std::shared_ptr<SyncReady>& sr : this->receivedSyncReadys) {
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
    this->lastTalkTime = ae->getAlicaClock()->now();

    ALICA_DEBUG_MSG("Sending Talk TID: " << sd.transitionID);

    this->syncModul->sendSyncTalk(talk);
}

void SynchronisationProcess::sendSyncReady()
{
    // send own row again to be sure
    sendTalk(myRow->getSyncData());

    SyncReady sr;
    sr.synchronisationID = this->synchronisation->getId();
    this->syncModul->sendSyncReady(sr);
}

/**
 * Before calling this method lock this mutex --> syncMutex (lock_guard<mutex> lock(syncMutex))
 */
bool SynchronisationProcess::isSyncComplete()
{
    // myRow needs to be acknowledged by all participants
    // every participant needs to believe in its condition
    // there must be at least one participant for every condition
    std::lock_guard<std::mutex> lock(rowOkMutex);
    this->rowsOK.clear();
    // collect participants
    for (int64_t transID : this->connectedTransitions) {
        SyncRow* foundRow = nullptr;

        //			Is not needed here
        //			lock_guard<mutex> lock(syncMutex);
        for (SyncRow* row : this->syncMatrix) {
            if (row->getSyncData().transitionID == transID && row->getSyncData().conditionHolds) {
                foundRow = row;
                break;
            }
        }
        if (foundRow == nullptr) // no robot for transition
        {
            return false;
        }

        if (find(this->rowsOK.begin(), this->rowsOK.end(), foundRow) == this->rowsOK.end()) {
            this->rowsOK.push_back(foundRow);
        }
    }
    //		check for acks in own row
    if (!this->myRow) {
        return false;
    }
    for (SyncRow* row : this->rowsOK) {
        essentials::IdentifierConstPtr tmp = row->getSyncData().robotID;
        if (std::find(this->myRow->getReceivedBy().begin(), this->myRow->getReceivedBy().end(), tmp) == this->myRow->getReceivedBy().end()) {
            return false;
        }
    }
    return true;
}

std::ostream& operator<<(std::ostream& s, const SynchronisationProcess& syncProc)
{
    s << std::endl;
    s << "Matrix:" << std::endl;

    for (SyncRow* row : syncProc.syncMatrix) {
        s << "Row: " << row->getSyncData().robotID << " "
          << std::to_string(row->getSyncData().transitionID) + " " + std::to_string(row->getSyncData().conditionHolds) << " " << row->getSyncData().ack
          << " RecvBy: ";
        for (essentials::IdentifierConstPtr robotID : row->getReceivedBy()) {
            s << robotID << ", ";
        }
        s << std::endl;
    }
    s << "ReceivedSyncreadys: ";
    for (const shared_ptr<SyncReady>& sr : syncProc.receivedSyncReadys) {
        s << sr->senderID << ", " << std::endl;
        ;
    }
    s << std::endl;
    return s;
}

} // namespace alica
/* namespace alica */
