#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/model/Synchronisation.h"
#include "engine/model/Transition.h"
#include "engine/teammanager/TeamManager.h"
#include <engine/syncmodule/SyncModule.h>
#include <engine/syncmodule/SynchronisationProcess.h>

namespace alica
{
using std::cerr;
using std::cout;
using std::endl;
using std::list;
using std::lock_guard;
using std::map;
using std::mutex;
using std::pair;
using std::shared_ptr;
using std::vector;

SyncModule::SyncModule(AlicaEngine* ae)
        : myId(nullptr)
        , ae(ae)
        , pr(nullptr)
        , running(false)
        , ticks(0)
        , communicator(nullptr)
{
}

SyncModule::~SyncModule()
{
    for (auto iter : this->synchSet) {
        delete iter.second;
    }
}
void SyncModule::init()
{
    lock_guard<mutex> lock(lomutex);
    this->ticks = 0;
    this->running = true;
    this->myId = ae->getTeamManager()->getLocalAgentID();
    this->pr = this->ae->getPlanRepository();
    this->communicator = this->ae->getCommunicator();
}
void SyncModule::close()
{
    this->running = false;
    cout << "SynchModul: Closed SynchModul" << endl;
}
void SyncModule::tick()
{
    list<SynchronisationProcess*> failedSyncs;
    lock_guard<mutex> lock(lomutex);
    for (auto iter : this->synchSet) {
        if (!iter.second->isValid(ticks)) {
            failedSyncs.push_back(iter.second);
        }
        ticks++;
        for (const SynchronisationProcess* s : failedSyncs) {
            delete this->synchSet[s->getSynchronisation()];
            // was without iter before
            this->synchSet.erase(s->getSynchronisation());
        }
    }
}
void SyncModule::setSynchronisation(const Transition* trans, bool holds)
{
    SynchronisationProcess* s;
    map<const Synchronisation*, SynchronisationProcess*>::iterator i = this->synchSet.find(trans->getSynchronisation());
    if (i != this->synchSet.end()) {
        i->second->setTick(this->ticks);
        i->second->changeOwnData(trans->getId(), holds);
    } else {
        s = new SynchronisationProcess(ae, myId, trans->getSynchronisation(), this);
        s->setTick(this->ticks);
        s->changeOwnData(trans->getId(), holds);
        lock_guard<mutex> lock(this->lomutex);
        synchSet.insert(pair<const Synchronisation*, SynchronisationProcess*>(trans->getSynchronisation(), s));
    }
}
void SyncModule::sendSyncTalk(SyncTalk& st)
{
    if (!this->ae->maySendMessages())
        return;
    st.senderID = this->myId;
    this->communicator->sendSyncTalk(st);
}
void SyncModule::sendSyncReady(SyncReady& sr)
{
    if (!this->ae->maySendMessages())
        return;
    sr.senderID = this->myId;
    communicator->sendSyncReady(sr);
}
void SyncModule::sendAcks(const std::vector<SyncData>& syncDataList) const
{
    if (!this->ae->maySendMessages())
        return;
    SyncTalk st;
    st.senderID = this->myId;
    st.syncData = syncDataList;
    this->communicator->sendSyncTalk(st);
}
void SyncModule::synchronisationDone(const Synchronisation* sync)
{
#ifdef SM_SUCCES
    cout << "SyncDONE in SYNCMODUL for synchronisationID: " << sync->getId() << endl;
#endif

#ifdef SM_SUCCES
    cout << "Remove synchronisationProcess object for synchronisationID: " << sync->getId() << endl;
#endif
    delete this->synchSet[sync];
    this->synchSet.erase(sync);
    this->synchronisations.push_back(sync);
#ifdef SM_SUCCES
    cout << "SM: SYNC TRIGGER TIME:" << this->ae->getAlicaClock()->now().inMilliseconds() << endl;
#endif
}

bool SyncModule::followTransition(const Transition *trans)
{
    list<const Synchronisation*>::iterator it = find(this->synchronisations.begin(), this->synchronisations.end(),
                                                     trans->getSynchronisation());
    if (it != this->synchronisations.end()) {
        this->synchronisations.remove(trans->getSynchronisation());
        return true;
    }
    return false;
}
void SyncModule::onSyncTalk(shared_ptr<SyncTalk> st)
{
    if (!this->running || st->senderID == this->myId)
        return;
    if (this->ae->getTeamManager()->isAgentIgnored(st->senderID))
        return;

#ifdef SM_SUCCES
    cout << "SyncModul:Handle Synctalk" << endl;
#endif

    std::vector<SyncData> toAck;
    for (const SyncData& sd : st->syncData) {
#ifdef SM_SUCCES
        cout << "SyncModul: TransID" << sd.transitionID << endl;
        cout << "SyncModul: RobotID" << sd.robotID << endl;
        cout << "SyncModul: Condition" << sd.conditionHolds << endl;
        cout << "SyncModul: ACK" << sd.ack << endl;
#endif

        const Transition* trans = this->pr->getTransitions().find(sd.transitionID);
        const Synchronisation* synchronisation = nullptr;

        if (trans != nullptr) {
            if (trans->getSynchronisation() != nullptr) {
                synchronisation = trans->getSynchronisation();
            } else {
                cerr << "SyncModul: Transition " << trans->getId() << " is not connected to a Synchronisation" << endl;
                return;
            }
        } else {
            cerr << "SyncModul: Could not find Element for Transition with ID: " << sd.transitionID << endl;
            return;
        }

        SynchronisationProcess* syncProc = nullptr;
        bool doAck = true;
        {
            lock_guard<mutex> lock(lomutex);
            map<const Synchronisation*, SynchronisationProcess*>::iterator i = this->synchSet.find(synchronisation);

            if (i != this->synchSet.end()) {
                syncProc = i->second;
                syncProc->integrateSyncTalk(st, this->ticks);
            } else {
                syncProc = new SynchronisationProcess(ae, this->myId, synchronisation, this);
                synchSet.insert(std::pair<const Synchronisation*, SynchronisationProcess*>(synchronisation, syncProc));
                doAck = syncProc->integrateSyncTalk(st, this->ticks);
            }
        }
        if (!sd.ack && *(st->senderID) == *(sd.robotID) && doAck) {
            toAck.push_back(sd);
        }
    }
    for (SyncData& sd : toAck) {
        sd.ack = true;
    }
    if (toAck.size() > 0) {
        sendAcks(toAck);
    }
}
void SyncModule::onSyncReady(shared_ptr<SyncReady> sr)
{
    if (!this->running || *(sr->senderID) == *(this->myId))
        return;
    if (this->ae->getTeamManager()->isAgentIgnored(sr->senderID))
        return;
    const Synchronisation* syncTrans = this->pr->getSynchronisations().find(sr->synchronisationID);

    if (syncTrans == nullptr) {
        cout << "SyncModul: Unable to find synchronisation " << sr->synchronisationID << " send by " << sr->senderID << endl;
        return;
    }
    {
        lock_guard<mutex> lock(lomutex);
        map<const Synchronisation*, SynchronisationProcess*>::iterator i = this->synchSet.find(syncTrans);
        if (i != this->synchSet.end()) {
            i->second->integrateSyncReady(sr);
        }
    }
}

} // namespace alica
