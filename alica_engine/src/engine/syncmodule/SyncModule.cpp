#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/PlanRepository.h"
#include "engine/TeamObserver.h"
#include "engine/containers/SyncData.h"
#include "engine/containers/SyncReady.h"
#include "engine/containers/SyncTalk.h"
#include "engine/model/SyncTransition.h"
#include "engine/model/Transition.h"
#include "engine/teammanager/TeamManager.h"
#include <engine/syncmodule/SyncModule.h>
#include <engine/syncmodule/Synchronisation.h>

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
        : _myId(nullptr)
        , _ae(ae)
        , _running(false)
        , _ticks(0)
{
}

SyncModule::~SyncModule()
{
    for (auto iter : _synchSet) {
        delete iter.second;
    }
}
void SyncModule::init()
{
    lock_guard<mutex> lock(_lomutex);
    _ticks = 0;
    _running = true;
    _myId = _ae->getTeamManager().getLocalAgentID();
}
void SyncModule::close()
{
    _running = false;
    cout << "SynchModul: Closed SynchModul" << endl;
}
void SyncModule::tick()
{
    list<Synchronisation*> failedSyncs;
    lock_guard<mutex> lock(_lomutex);
    for (auto iter : _synchSet) {
        if (!iter.second->isValid(_ticks)) {
            failedSyncs.push_back(iter.second);
        }
        _ticks++;
        for (const Synchronisation* s : failedSyncs) {
            delete _synchSet[s->getSyncTransition()];
            // was without iter before
            _synchSet.erase(s->getSyncTransition());
        }
    }
}
void SyncModule::setSynchronisation(const Transition* trans, bool holds)
{
    Synchronisation* s;
    map<const SyncTransition*, Synchronisation*>::iterator i = _synchSet.find(trans->getSyncTransition());
    if (i != _synchSet.end()) {
        i->second->setTick(_ticks);
        i->second->changeOwnData(trans->getId(), holds);
    } else {
        s = new Synchronisation(_ae, _myId, trans->getSyncTransition(), this);
        s->setTick(_ticks);
        s->changeOwnData(trans->getId(), holds);
        lock_guard<mutex> lock(_lomutex);
        _synchSet.insert(pair<const SyncTransition*, Synchronisation*>(trans->getSyncTransition(), s));
    }
}
void SyncModule::sendSyncTalk(SyncTalk& st)
{
    if (!_ae->maySendMessages())
        return;
    st.senderID = _myId;
    _ae->getCommunicator().sendSyncTalk(st);
}
void SyncModule::sendSyncReady(SyncReady& sr)
{
    if (!_ae->maySendMessages())
        return;
    sr.senderID = _myId;
    _ae->getCommunicator().sendSyncReady(sr);
}
void SyncModule::sendAcks(const std::vector<SyncData>& syncDataList) const
{
    if (!_ae->maySendMessages())
        return;
    SyncTalk st;
    st.senderID = _myId;
    st.syncData = syncDataList;
    _ae->getCommunicator().sendSyncTalk(st);
}
void SyncModule::synchronisationDone(const SyncTransition* st)
{
#ifdef SM_SUCCES
    cout << "SyncDONE in SYNCMODUL for synctransID: " << st->getId() << endl;
#endif

#ifdef SM_SUCCES
    cout << "Remove synchronisation object for syntransID: " << st->getId() << endl;
#endif
    delete _synchSet[st];
    _synchSet.erase(st);
    _synchedTransitions.push_back(st);
#ifdef SM_SUCCES
    cout << "SM: SYNC TRIGGER TIME:" << _ae->getAlicaClock().now().inMilliseconds() << endl;
#endif
}

bool SyncModule::followSyncTransition(const Transition* trans)
{
    list<const SyncTransition*>::iterator it = find(_synchedTransitions.begin(), _synchedTransitions.end(), trans->getSyncTransition());
    if (it != _synchedTransitions.end()) {
        _synchedTransitions.remove(trans->getSyncTransition());
        return true;
    }
    return false;
}
void SyncModule::onSyncTalk(shared_ptr<SyncTalk> st)
{
    if (!_running || st->senderID == _myId)
        return;
    if (_ae->getTeamManager().isAgentIgnored(st->senderID))
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

        const Transition* trans = _ae->getPlanRepository().getTransitions().find(sd.transitionID);
        const SyncTransition* syncTrans = nullptr;

        if (trans != nullptr) {
            if (trans->getSyncTransition() != nullptr) {
                syncTrans = trans->getSyncTransition();
            } else {
                cerr << "SyncModul: Transition " << trans->getId() << " is not connected to a SyncTransition" << endl;
                return;
            }
        } else {
            cerr << "SyncModul: Could not find Element for Transition with ID: " << sd.transitionID << endl;
            return;
        }

        Synchronisation* sync = nullptr;
        bool doAck = true;
        {
            lock_guard<mutex> lock(_lomutex);
            map<const SyncTransition*, Synchronisation*>::iterator i = _synchSet.find(syncTrans);

            if (i != _synchSet.end()) {
                sync = i->second;
                sync->integrateSyncTalk(st, _ticks);
            } else {
                sync = new Synchronisation(_ae, _myId, syncTrans, this);
                _synchSet.insert(pair<const SyncTransition*, Synchronisation*>(syncTrans, sync));
                doAck = sync->integrateSyncTalk(st, _ticks);
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
    if (!_running || *(sr->senderID) == *(_myId))
        return;
    if (_ae->getTeamManager().isAgentIgnored(sr->senderID))
        return;
    const SyncTransition* syncTrans = _ae->getPlanRepository().getSyncTransitions().find(sr->syncTransitionID);

    if (syncTrans == nullptr) {
        cout << "SyncModul: Unable to find synchronisation " << sr->syncTransitionID << " send by " << sr->senderID << endl;
        return;
    }
    {
        lock_guard<mutex> lock(_lomutex);
        map<const SyncTransition*, Synchronisation*>::iterator i = _synchSet.find(syncTrans);
        if (i != _synchSet.end()) {
            i->second->integrateSyncReady(sr);
        }
    }
}

} // namespace alica
