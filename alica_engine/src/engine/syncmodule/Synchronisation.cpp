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

namespace alica
{

Synchronisation::Synchronisation(AlicaEngine* ae)
    : myID(nullptr)
{
    this->ae = ae;
    this->syncModul = nullptr;
    this->syncTransition = nullptr;
    this->readyForSync = false;
    this->lastTick = 0;
    this->runningPlan = nullptr;
    this->myRow = nullptr;
    this->lastTalkData = nullptr;
}

Synchronisation::Synchronisation(AlicaEngine* ae, const supplementary::AgentID* myID, const SyncTransition* st, SyncModule* sm)
{
    this->ae = ae;
    this->syncTransition = st;
    this->myID = myID;
    this->syncStartTime = ae->getAlicaClock()->now();
    for (const Transition* t : st->getInSync()) {
        connectedTransitions.push_back(t->getId());
    }
    this->syncModul = sm;
    this->runningPlan = nullptr;
    this->myRow = nullptr;
    this->readyForSync = false;
    this->lastTick = 0;
    this->lastTalkData = nullptr;
}

Synchronisation::~Synchronisation()
{
    for (auto row : syncMatrix) {
        delete row;
    }
}

const SyncTransition* Synchronisation::getSyncTransition() const
{
    return syncTransition;
}

void Synchronisation::setTick(uint64_t now)
{
    this->lastTick = now;
}

void Synchronisation::changeOwnData(int64_t transitionID, bool conditionHolds)
{
#ifdef SM_MISC
    cout << "CHOD: ElapsedTime: " << (ae->getAlicaClock()->now() - this->syncStartTime) << endl;
#endif

    if (!conditionHolds) {
        // my condition does not hold => not ready for syncing
        this->readyForSync = false;
    }
    SyncData* sd = new SyncData();
    sd->robotID = this->myID;
    sd->transitionID = transitionID;
    sd->conditionHolds = conditionHolds;
    sd->ack = false;

    bool maySendTalk = true;

    {
        std::lock_guard<mutex> lock(syncMutex);
        if (myRow != nullptr) {
            if (/*sd->ack != myRow->getSyncData()->ack
						||*/ sd->conditionHolds != myRow->getSyncData()->conditionHolds ||
                *(sd->robotID) != *(myRow->getSyncData()->robotID) || sd->transitionID != myRow->getSyncData()->transitionID) {
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
                delete sd;
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
            cout << "ChangedOwnData: Synctrans " << this->syncTransition->getId() << " ready" << endl;
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
bool Synchronisation::isValid(uint64_t curTick)
{
    bool stillActive = (this->lastTick + 2 >= curTick);

    if (!stillActive) {
        // notify others if i am part of the synchronisation already (i.e. have an own row)
        if (myRow != nullptr) {
            if (myRow->getSyncData() != nullptr) {
                myRow->getSyncData()->conditionHolds = false;

                sendTalk(myRow->getSyncData());
            }
        }

        return false;
    }

    AlicaTime now = ae->getAlicaClock()->now();

    if (this->lastTalkTime != AlicaTime::zero()) // talked already
    {
#ifdef SM_FAILURE
        cout << "TestTimeOut on Sync: " << this->syncTransition->getId() << endl;
#endif
        if ((now > this->syncTransition->getTalkTimeOut() + this->lastTalkTime) && !this->readyForSync) {
            if (this->myRow != nullptr) {
                sendTalk(this->myRow->getSyncData());
            }
        }
    }

#ifdef SM_FAILURE
    cout << "Synchronisation: TestTimeOut(): syncStarTime " << this->syncStartTime << endl;
#endif

    if (this->syncTransition->isFailOnSyncTimeOut()) {
        if (now > this->syncTransition->getSyncTimeOut() + this->syncStartTime) {
#ifdef SM_FAILURE
            cout << "Synchronisation: TestTimeOut() sync failed" << endl;
#endif
            return false;
        }
    }

    return true;
}

bool Synchronisation::integrateSyncTalk(std::shared_ptr<SyncTalk> talk, uint64_t curTick)
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

#ifdef SM_MESSAGES
    cout << "Integrate synctalk in synchronisation" << endl;
    cout << "ST: ElapsedTime: " << (ae->getAlicaClock()->now() - this->syncStartTime) << endl;
#endif
    for (SyncData* sd : talk->syncData) {
#ifdef SM_MESSAGES
        cout << "syncdata for transID: " << sd->transitionID << endl;
#endif
        std::lock_guard<mutex> lock(syncMutex);
        {
            SyncRow* rowInMatrix = nullptr;
            for (SyncRow* row : this->syncMatrix) {
#ifdef SM_MESSAGES
                cout << "ROW SD: " << row->getSyncData()->robotID << " " << row->getSyncData()->transitionID << " " << row->getSyncData()->conditionHolds << " "
                     << row->getSyncData()->ack << endl;
                cout << "CUR SD: " << sd->robotID << " " << sd->transitionID << " " << sd->conditionHolds << " " << sd->ack << endl;
#endif

                if (/*sd->ack == row->getSyncData()->ack
							&&*/ sd->conditionHolds == row->getSyncData()->conditionHolds &&
                    *(sd->robotID) == *(row->getSyncData()->robotID) && sd->transitionID == row->getSyncData()->transitionID) {
                    rowInMatrix = row;
                    break;
                }
            }

            // if(rowInMatrix != null)
            if (rowInMatrix == nullptr) {
#ifdef SM_MESSAGES
                cout << "NEW MATRIX row" << endl;
#endif
                SyncRow* newRow = new SyncRow(sd);
                newRow->getReceivedBy().push_back(talk->senderID);

                syncMatrix.push_back(newRow);
            } else {
#ifdef SM_MESSAGES
                cout << "Received by: " << talk->senderID << endl;
#endif
                rowInMatrix->getReceivedBy().push_back(talk->senderID);
            }
            if (isSyncComplete()) {
#ifdef SM_MESSAGES
                cout << "IntegrateSyncTalk: Synctrans " << this->syncTransition->getId() << " ready" << endl;
#endif
                sendSyncReady();
                this->readyForSync = true;
            } else {
                // always reset this in case someone revokes his commitment
                this->readyForSync = false;
            }
#ifdef SM_MESSAGES
            cout << "Matrix: IntSyncTalk" << endl;
            printMatrix();
#endif

            // late acks...
            if (this->readyForSync) {
                if (allSyncReady()) {
#ifdef SM_SUCCESSS
                    Console.WriteLine("SyncDONE in Synchronisation (IntTalk): elapsed time: " + (ae->getAlicaClock()->now()) - this.syncStartTime);
#endif
                    // notify syncmodul
                    this->syncModul->synchronisationDone(this->syncTransition);
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
    for (std::shared_ptr<SyncReady>& sr : this->receivedSyncReadys) {
        if (*(sr->senderID) == *(ready->senderID)) {
            found = true;
            break;
        }
    }

    if (!found) {
        this->receivedSyncReadys.push_back(ready);
    }
#ifdef SM_MESSAGES
    cout << "Matrix: IntSyncReady" << endl;
    printMatrix();
#endif

    // check if all robots are ready
    if (this->readyForSync) {
        if (allSyncReady()) {
        // notify syncModul
#ifdef SM_SUCCESS
            cout << "SyncDONE in Synchronisation (IntReady): elapsed time: " << (ae->getAlicaClock()->now() - this->syncStartTime) << endl;
#endif
            this->syncModul->synchronisationDone(this->syncTransition);
        }
    }
}

void Synchronisation::setSyncTransition(const SyncTransition* syncTransition)
{
    this->syncTransition = syncTransition;
}

bool Synchronisation::allSyncReady()
{
    // test if all robots who acknowledged myRow have sent a SyncReady
    for (AgentIDConstPtr robotID : this->myRow->getReceivedBy()) {
        if (*robotID != *myID) // we do not necessarily need an ack from ourselves
        {
            bool foundRobot = false;
            for (std::shared_ptr<SyncReady>& sr : this->receivedSyncReadys) {
                if (*(sr->senderID) == *(robotID)) {
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

void Synchronisation::printMatrix()
{
    cout << endl;
    cout << "Matrix:" << endl;

    std::lock_guard<std::mutex> lock(syncMutex);
    {
        for (SyncRow* row : this->syncMatrix) {
            std::cout << "Row: " << row->getSyncData()->robotID << " "
                      << to_string(row->getSyncData()->transitionID) + " " + std::to_string(row->getSyncData()->conditionHolds) << " "
                      << row->getSyncData()->ack << " RecvBy: ";
            for (AgentIDConstPtr robotID : row->getReceivedBy()) {
                std::cout << *robotID << ", ";
            }
            cout << endl;
        }
        std::cout << "ReceivedSyncreadys: ";
        for (shared_ptr<SyncReady> sr : this->receivedSyncReadys) {
            std::cout << sr->senderID << ", " << std::endl;
            ;
        }
    }
    std::cout << std::endl;
}

void Synchronisation::sendTalk(SyncData* sd)
{
    SyncTalk talk;
    talk.syncData.push_back(sd);
    this->lastTalkTime = ae->getAlicaClock()->now();

#ifdef SM_MESSAGES
    cout << "Sending Talk TID: " << sd->transitionID << endl;
#endif

    this->syncModul->sendSyncTalk(talk);
}

void Synchronisation::sendSyncReady()
{
    // send own row again to be sure
    sendTalk(myRow->getSyncData());

    SyncReady sr;
    sr.syncTransitionID = this->syncTransition->getId();
    this->syncModul->sendSyncReady(sr);
}

/**
 * Before calling this method lock this mutex --> syncMutex (lock_guard<mutex> lock(syncMutex))
 */
bool Synchronisation::isSyncComplete()
{
    // myRow needs to be acknowledged by all participants
    // every participant needs to believe in its condition
    // there must be at least one participant for every condition
    lock_guard<mutex> lock(rowOkMutex);
    this->rowsOK.clear();
    // collect participants
    for (int64_t transID : this->connectedTransitions) {
        SyncRow* foundRow = nullptr;

        //			Is not needed here
        //			lock_guard<mutex> lock(syncMutex);
        for (SyncRow* row : this->syncMatrix) {
            if (row->getSyncData()->transitionID == transID && row->getSyncData()->conditionHolds) {
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
        auto tmp = row->getSyncData()->robotID;
        if (find_if(this->myRow->getReceivedBy().begin(), this->myRow->getReceivedBy().end(),
                    [&tmp](const supplementary::AgentID* id) { return *tmp == *id; }) == this->myRow->getReceivedBy().end()) {
            return false;
        }
    }
    return true;
}

} // namespace alica
/* namespace alica */
