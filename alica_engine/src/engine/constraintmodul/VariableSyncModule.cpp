#include "engine/constraintmodul/VariableSyncModule.h"

#include "engine/IAlicaCommunication.h"
#include "engine/IAlicaTimer.h"
#include "engine/TeamObserver.h"
#include "engine/constraintmodul/ResultEntry.h"
#include "engine/model/Variable.h"
#include "engine/teammanager/TeamManager.h"
#include <algorithm>
#include <cmath>

#include <assert.h>
#include <functional>

namespace alica
{
VariableSyncModule::VariableSyncModule(AlicaEngine* ae)
        : _ae(ae)
        , _running(false)
        , _timer(nullptr)
        , _distThreshold(0)
        , _ttl4Communication(AlicaTime::zero())
        , _ttl4Usage(AlicaTime::zero())
        , _ownResults(nullptr)
{
}

VariableSyncModule::~VariableSyncModule() {}

void VariableSyncModule::init()
{
    assert(!_running);
    if (_running) {
        return;
    }
    auto reloadFunctionPtr = std::bind(&VariableSyncModule::reload, this, std::placeholders::_1);
    _ae->subscribe(reloadFunctionPtr);
    reload(_ae->getConfig());
    _running = true;
}

void VariableSyncModule::reload(const YAML::Node& config)
{
    if (_running) {
        return;
    }
    bool communicationEnabled = config["Alica"]["CSPSolving"]["EnableCommunication"].as<bool>();
    _ttl4Communication = AlicaTime::milliseconds(config["Alica"]["CSPSolving"]["SeedTTL4Communication"].as<long>());
    _ttl4Usage = AlicaTime::milliseconds(config["Alica"]["CSPSolving"]["SeedTTL4Usage"].as<long>());
    _distThreshold = config["Alica"]["CSPSolving"]["SeedMergingThreshold"].as<double>();

    AgentId ownId = _ae->getTeamManager().getLocalAgentID();
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _store.emplace_back(new ResultEntry(ownId));
        _ownResults = _store.back().get();
    }

    _publishData.senderID = ownId;

    if (communicationEnabled) {
        double communicationFrequency = config["Alica"]["CSPSolving"]["CommunicationFrequency"].as<double>();
        AlicaTime interval = AlicaTime::seconds(1.0 / communicationFrequency);
        if (_timer == nullptr) {
            _timer = _ae->getTimerFactory().createTimer(std::move(std::bind(&VariableSyncModule::publishContent, this)), interval);
        }
    }
}

void VariableSyncModule::close()
{
    _running = false;
    if (_timer) {
        _timer.reset();
    }
}

void VariableSyncModule::clear()
{
    for (std::unique_ptr<ResultEntry>& r : _store) {
        r->clear();
    }
}

void VariableSyncModule::onSolverResult(const SolverResult& msg)
{
    if (!_running || msg.senderID == _ownResults->getId() || _ae->getTeamManager().isAgentIgnored(msg.senderID)) {
        return;
    }

    ResultEntry* re = nullptr;
    for (std::unique_ptr<ResultEntry>& r : _store) {
        if ((r->getId()) == (msg.senderID)) {
            re = r.get();
            break;
        }
    }

    if (re == nullptr) {
        std::unique_ptr<ResultEntry> new_entry(new ResultEntry(msg.senderID));
        std::lock_guard<std::mutex> lock(_mutex);
        auto agent_sorted_loc = std::upper_bound(_store.begin(), _store.end(), new_entry,
                [](const std::unique_ptr<ResultEntry>& a, const std::unique_ptr<ResultEntry>& b) { return (a->getId() < b->getId()); });
        agent_sorted_loc = _store.insert(agent_sorted_loc, std::move(new_entry));
        re = (*agent_sorted_loc).get();
    }

    AlicaTime now = _ae->getAlicaClock().now();
    for (const SolverVar& sv : msg.vars) {
        Variant v;
        variant::loadFrom(sv.value, v);
        re->addValue(sv.id, v, now);
    }
}

void VariableSyncModule::publishContent()
{
    if (!_running) {
        return;
    }
    if (!_ae->maySendMessages()) {
        return;
    }

    _publishData.vars.clear();
    AlicaTime now = _ae->getAlicaClock().now();
    _ownResults->getCommunicatableResults(now - _ttl4Communication, _publishData.vars);
    if (_publishData.vars.empty()) {
        return;
    }
    _ae->getCommunicator().sendSolverResult(_publishData);
}

void VariableSyncModule::postResult(int64_t vid, Variant result)
{
    _ownResults->addValue(vid, result, _ae->getAlicaClock().now());
}

VariableSyncModule::VotedSeed::VotedSeed(std::vector<Variant>&& vs)
        : _values(std::move(vs))
        , _supporterCount(_values.size())
        , _hash(0)
        , _totalSupCount(0)
{
    assert(_values.size() == _supporterCount.size());
    for (const Variant& v : _values) {
        if (variant::isSet(v)) {
            ++_totalSupCount;
        }
    }
}

VariableSyncModule::VotedSeed::VotedSeed(VotedSeed&& o)
        : _values(std::move(o._values))
        , _supporterCount(std::move(o._supporterCount))
        , _hash(o._hash)
        , _totalSupCount(o._totalSupCount)
{
}

VariableSyncModule::VotedSeed& VariableSyncModule::VotedSeed::operator=(VotedSeed&& o)
{
    _values = std::move(o._values);
    _supporterCount = std::move(o._supporterCount);
    _hash = o._hash;
    _totalSupCount = o._totalSupCount;
    return *this;
}

bool VariableSyncModule::VotedSeed::takeVector(const std::vector<Variant>& v, const std::vector<Interval<double>>& limits, double distThreshold)
{
    int nans = 0;
    const int dim = static_cast<int>(v.size());
    double distSqr = 0;

    for (int i = 0; i < dim; ++i) {
        if (std::holds_alternative<double>(v[i])) {
            if (std::holds_alternative<double>(_values[i])) {
                double d = std::get<double>(v[i]);
                if (!std::isnan(d)) {
                    double cur = std::get<double>(_values[i]);
                    double dist = (d - cur) * (d - cur);
                    double size = limits[i].size();
                    if (size > 0.0) {
                        dist /= size * size;
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
        return true; // silently absorb a complete NaN vector
    }
    if (distSqr / (dim - nans) < distThreshold) { // merge
        for (int i = 0; i < dim; ++i) {
            if (std::holds_alternative<double>(v[i])) {
                double d = std::get<double>(v[i]);
                if (!std::isnan(d)) {
                    if (std::holds_alternative<double>(_values[i])) {
                        double nv = std::get<double>(_values[i]) * _supporterCount[i] + d;
                        ++_supporterCount[i];
                        nv /= _supporterCount[i];
                        _values[i] = nv;
                    } else {
                        _supporterCount[i] = 1;
                        _values[i] = d;
                    }
                    ++_totalSupCount;
                }
            }
        }
        // recalc hash:
        _hash = 0;
        for (const Variant v : _values) {
            if (std::holds_alternative<double>(v)) {
                _hash += std::get<double>(v);
            }
        }
        return true;
    }
    return false;
}
} /* namespace alica */
