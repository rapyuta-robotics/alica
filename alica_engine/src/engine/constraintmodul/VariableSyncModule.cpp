//#define RS_DEBUG
#include "engine/constraintmodul/VariableSyncModule.h"

#include "SystemConfig.h"
#include "engine/AlicaEngine.h"
#include "engine/IAlicaCommunication.h"
#include "engine/teammanager/TeamManager.h"
#include "engine/TeamObserver.h"
#include "engine/constraintmodul/ResultEntry.h"
#include "engine/model/Variable.h"
#include <algorithm>
#include <cmath>

#include <assert.h>


namespace alica {
VariableSyncModule::VariableSyncModule(AlicaEngine* ae)
        : _ae(ae)
        , _running(false)
        , _timer(nullptr)
        , _distThreshold(0)
        , _communicator(nullptr)
        , _ttl4Communication(AlicaTime::zero())
        , _ttl4Usage(AlicaTime::zero())
        {}

VariableSyncModule::~VariableSyncModule() {
    delete _timer;
}

void VariableSyncModule::init() {
    assert(!_running);
    if (_running) {
        return;
    }
    _running = true;
    supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    bool communicationEnabled = (*sc)["Alica"]->get<bool>("Alica", "CSPSolving", "EnableCommunication", NULL);
    _ttl4Communication = AlicaTime::milliseconds((*sc)["Alica"]->get<long>("Alica", "CSPSolving", "SeedTTL4Communication", NULL));
    _ttl4Usage = AlicaTime::milliseconds((*sc)["Alica"]->get<long>("Alica", "CSPSolving", "SeedTTL4Usage", NULL));
    _distThreshold = (*sc)["Alica"]->get<double>("Alica", "CSPSolving", "SeedMergingThreshold", NULL);

    AgentIDPtr ownId = _ae->getTeamManager()->getLocalAgentID();
    _ownResults = ResultEntry(ownId);
    _publishData.senderID = ownId;
   

    if (communicationEnabled) {
        _communicator = _ae->getCommunicator();
        double communicationFrequency = (*sc)["Alica"]->get<double>("Alica", "CSPSolving", "CommunicationFrequency", NULL);
        AlicaTime interval = AlicaTime::seconds(1.0 / communicationFrequency);
        if(_timer == nullptr) {
            _timer = new supplementary::NotifyTimer<VariableSyncModule>(interval.inMilliseconds(), &VariableSyncModule::publishContent, this);
        }
        _timer->start();
    }
}

void VariableSyncModule::close() {
    _running = false;
    if (_timer) {
        _timer->stop();
    }
}

void VariableSyncModule::clear() {
    for (ResultEntry& r : _store) {
        r.clear();
    }
}

void VariableSyncModule::onSolverResult(const SolverResult& msg) {
    if (*(msg.senderID) == *_store[0].getId()) {
        return;
    }
    if (_ae->getTeamManager()->isAgentIgnored(msg.senderID)) {
        return;
    }
    ResultEntry* re = nullptr;
    
    for (ResultEntry& r : _store) {
        if (*(r.getId()) == *(msg.senderID)) {
            re = &r;
            break;
        }
    }
    if (re == nullptr) {
        lock_guard<std::mutex> lock(_mutex);
        _store.emplace_back(msg.senderID);
        re = &_store.back();
    }
    AlicaTime now = _ae->getAlicaClock()->now();
    for (const SolverVar& sv : msg.vars) {
        Variant v;
        v.loadFrom(sv.value);
        re->addValue(sv.id, v, now);
    }
}

void VariableSyncModule::publishContent() {
    if (!_running) {
        return;
    }
    if (!_ae->maySendMessages()) {
        return;
    }

    _publishData.vars.clear();
    AlicaTime now = _ae->getAlicaClock()->now();
    _ownResults.getCommunicatableResults(now - _ttl4Communication, _publishData.vars);
    if (_publishData.vars.empty()) {
        return;
    }
    _communicator->sendSolverResult(_publishData);
}

void VariableSyncModule::postResult(int64_t vid, Variant result) {
    _ownResults.addValue(vid, result, _ae->getAlicaClock()->now());
}

int VariableSyncModule::getSeeds(const VariableSet& query, const std::vector<double>& limits, std::vector<Variant>& o_seeds) const {

    const int dim = query.size();
    //TODO: use only stack memory for low dimensionality
    std::vector<double> scaling(dim);
    std::vector<Variant> vec(dim);

    std::vector<VotedSeed> seeds;
    
    for (int i = 0; i < dim; ++i) {
        scaling[i] = limits[i*2+1] - limits[i*2];
        scaling[i] *= scaling[i];  // Sqr it for dist calculation speed up
    }
    AlicaTime earliest = _ae->getAlicaClock()->now() - _ttl4Usage;
    //		cout << "VSM: Number of Seeds in Store: " << this->store.size() << endl;
    if(_ownResults.getValues(query,earliest, vec)) {
        seeds.emplace_back(std::move(vec));
    }

    lock_guard<std::mutex> lock(_mutex);

    for (const ResultEntry& re : _store) {
        
        bool any = re.getValues(query, earliest, vec);
        if (!any) {
            continue;
        }
        bool found = false;
        for (VotedSeed& s : seeds) {
            if (s.takeVector(vec, scaling, _distThreshold)) {
                found = true;
                break;
            }
        }
        if (!found) {
            seeds.emplace_back(std::move(vec));
        }
    }
#ifdef RS_DEBUG
    std::cout << "RS: Generated " << seeds.size() << "seeds" << std::endl;
    for (int i = 0; i < seeds.size(); ++i) {
        cout << "Seed " << i << ": ";  // (sup:{1}): ",i);
        for (auto j = 0; j < dim; ++j) {
            cout << seeds[i].values[j] << "\t";
        }
        cout << endl;
    }
#endif

    int maxNum = std::min(static_cast<int>(seeds.size()), dim);

    std::sort(seeds.begin(), seeds.end(),[](const VotedSeed& a, const VotedSeed& b) {
        if (a._totalSupCount != b._totalSupCount) {
            return a._totalSupCount > b._totalSupCount;
        } else {
            /*
            if (a.values == nullptr || a.values.size() == 0)
                return true;
            if (b.values == nullptr || b.values.size() == 0)
                return false;
            */
            return a._hash > b._hash;
            
        }
    });
    int i=0;
    o_seeds.resize(dim*maxNum);
    for(const VotedSeed& vs : seeds) {
        memcpy(&*(o_seeds.begin()+i),&*vs._values.begin(), sizeof(Variant)*dim);
        i+=dim;
    }

    return maxNum;
}

VariableSyncModule::VotedSeed::VotedSeed(std::vector<Variant>&& vs) 
    : _values(std::move(vs))
    , _supporterCount(_values.size())
    , _hash(0)
    , _totalSupCount(0)
{
    assert(_values.size() == _supporterCount.size());
    for (const Variant& v : _values) {
        if (v.isSet()) {
            ++_totalSupCount;
        }
    }
}

VariableSyncModule::VotedSeed::VotedSeed(VotedSeed&& o)
    : _values(std::move(o._values))
    , _supporterCount(std::move(o._supporterCount))
    , _hash(o._hash)
    , _totalSupCount(o._totalSupCount)
{}

VariableSyncModule::VotedSeed& VariableSyncModule::VotedSeed::operator=(VotedSeed&& o) {
    _values =  std::move(o._values);
    _supporterCount =  std::move(o._supporterCount);
    _hash = o._hash;
    _totalSupCount = o._totalSupCount;
    return *this;
}

bool VariableSyncModule::VotedSeed::takeVector(const std::vector<Variant>& v, const std::vector<double>& scaling, double distThreshold) {

    int nans = 0;
    const int dim = static_cast<int>(v.size());
    double distSqr = 0;

    for (int i = 0; i < dim; ++i) {
        if(v[i].isDouble()) {
            if(_values[i].isDouble()) {
                double d = v[i].getDouble();
                if(!std::isnan(d) ) {
                    double cur = _values[i].getDouble();
                    double dist = (d-cur)*(d-cur);
                    if(scaling[i] > 0.0) {
                        dist /= scaling[i];
                    }
                    distSqr += dist;
                } else {
                    ++nans;
                }
            }
        } else {
            ++nans;
        }
    }

    if (dim == nans) {
        return true;  // silently absorb a complete NaN vector
    }
    if (distSqr / (dim - nans) < distThreshold) { //merge
        for (int i = 0; i < dim; ++i) {
            if(v[i].isDouble()) {
                double d = v[i].getDouble();
                if(!std::isnan(d)) {
                    if(_values[i].isDouble()) {
                        double nv = _values[i].getDouble() * _supporterCount[i] + d;
                        ++_supporterCount[i];
                        nv /= _supporterCount[i];
                        _values[i].setDouble(nv);
                    } else {
                        _supporterCount[i] = 1;
                        _values[i].setDouble(d);
                    }
                    ++_totalSupCount;
                }
            }
        }
        //recalc hash:
        _hash = 0;
        for (const Variant v : _values) {
            if (v.isDouble()) {
                _hash += v.getDouble();
            }
        }
        return true;
    }
    return false;
}
} /* namespace alica */
