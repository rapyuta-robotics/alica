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

SyncModule::SyncModule(const AlicaEngine* ae)
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
    cout << "SM: Closed SynchModul" << endl;
}
void SyncModule::tick()
{
    list<SynchronisationProcess*> failedSyncs;
    lock_guard<mutex> lock(_lomutex);
    for (auto iter : _synchSet) {
        if (!iter.second->isValid(_ticks)) {
            failedSyncs.push_back(iter.second);
        }
        _ticks++;
        for (const SynchronisationProcess* s : failedSyncs) {
            delete _synchSet[s->getSynchronisation()];
            _synchSet.erase(s->getSynchronisation());
        }
    }
}
void SyncModule::setSynchronisation(const Transition* trans, bool holds)
{
    SynchronisationProcess* s;
    map<const Synchronisation*, SynchronisationProcess*>::iterator i = _synchSet.find(trans->getSynchronisation());
    if (i != _synchSet.end()) {
        i->second->setTick(_ticks);
        i->second->changeOwnData(trans->getId(), holds);
    } else {
        s = new SynchronisationProcess(_ae, _myId, trans->getSynchronisation(), this);
        s->setTick(_ticks);
        s->changeOwnData(trans->getId(), holds);
        lock_guard<mutex> lock(_lomutex);
        _synchSet.insert(pair<const Synchronisation*, SynchronisationProcess*>(trans->getSynchronisation(), s));
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
void SyncModule::synchronisationDone(const Synchronisation* sync)
{
#ifdef SM_SUCCESS
    cout << "SM: Sync done for synchronisationID: " << sync->getId() << endl;
#endif

#ifdef SM_SUCCESS
    cout << "SM: Remove synchronisationProcess object for synchronisationID: " << sync->getId() << endl;
#endif
    delete _synchSet[sync];
    _synchSet.erase(sync);
    _synchronisations.push_back(sync);
#ifdef SM_SUCCESS
    cout << "SM: SYNC TRIGGER TIME: " << _ae->getAlicaClock().now().inMilliseconds() << endl;
#endif
}

bool SyncModule::followSyncedTransition(const Transition *trans)
{
    list<const Synchronisation*>::iterator it = find(_synchronisations.begin(), _synchronisations.end(), trans->getSynchronisation());
    if (it != _synchronisations.end()) {
        _synchronisations.remove(trans->getSynchronisation());
        return true;
    }
    return false;
}
void SyncModule::onSyncTalk(shared_ptr<SyncTalk> st)
{
    if (!_running || st->senderID == _myId || _ae->getTeamManager().isAgentIgnored(st->senderID)) {
        return;
    }
#ifdef SM_SUCCESS
    cout << "SM: Handle SyncTalk" << endl;
#endif

    std::vector<SyncData> toAck;
    for (const SyncData& sd : st->syncData) {
#ifdef SM_SUCCESS
        cout << "SM: " << sd << endl;
#endif

        const Transition* trans = _ae->getPlanRepository().getTransitions().find(sd.transitionID);
        const Synchronisation* synchronisation = nullptr;

        if (trans != nullptr) {
            if (trans->getSynchronisation() != nullptr) {
                synchronisation = trans->getSynchronisation();
            } else {
                cerr << "SM: Transition " << trans->getId() << " is not connected to a Synchronisation" << endl;
                return;
            }
        } else {
            cerr << "SM: Could not find Transition with ID: " << sd.transitionID << endl;
            return;
        }

        SynchronisationProcess* syncProc = nullptr;
        bool doAck = true;
        {
            lock_guard<mutex> lock(_lomutex);
            map<const Synchronisation*, SynchronisationProcess*>::iterator i = _synchSet.find(synchronisation);

            if (i != _synchSet.end()) {
                syncProc = i->second;
                syncProc->integrateSyncTalk(st, _ticks);
            } else {
                syncProc = new SynchronisationProcess(_ae, _myId, synchronisation, this);
                _synchSet.insert(pair<const Synchronisation*, SynchronisationProcess*>(synchronisation, syncProc));
                doAck = syncProc->integrateSyncTalk(st, _ticks);
            }
        }
        if (!sd.ack && *(st->senderID) == *(sd.agentID) && doAck) {
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
    if (!_running || sr->senderID == _myId || _ae->getTeamManager().isAgentIgnored(sr->senderID)) {
        return;
    }

    const Synchronisation* synchronisation = _ae->getPlanRepository().getSynchronisations().find(sr->synchronisationID);

    if (synchronisation == nullptr) {
        cout << "SM: Unable to find synchronisation " << sr->synchronisationID << " send by " << sr->senderID << endl;
        return;
    }
    {
        lock_guard<mutex> lock(_lomutex);
        map<const Synchronisation*, SynchronisationProcess*>::iterator i = _synchSet.find(synchronisation);
        if (i != _synchSet.end()) {
            i->second->integrateSyncReady(sr);
        }
    }
}

} // namespace alica
