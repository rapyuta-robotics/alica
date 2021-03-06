#pragma once
//#define RS_DEBUG
#include "engine/AlicaEngine.h"
#include "engine/constraintmodul/ResultEntry.h"
#include "engine/containers/SolverResult.h"

#include <alica_solver_interface/Interval.h>
#include <essentials/NotifyTimer.hpp>

#include <vector>

namespace alica
{
class Variable;
class ResultEntry;
class IAlicaCommunication;
class AlicaEngine;

class VariableSyncModule
{
public:
    VariableSyncModule(AlicaEngine* ae);
    ~VariableSyncModule();

    void init();
    void close();
    void clear();
    void onSolverResult(const SolverResult& msg);
    void publishContent();
    void postResult(int64_t vid, Variant result);
    void reload(const YAML::Node& config);

    template <typename VarType>
    int getSeeds(const std::vector<VarType*>& query, const std::vector<Interval<double>>& limits, std::vector<Variant>& o_seeds) const;
    VariableSyncModule(const VariableSyncModule&) = delete;
    VariableSyncModule(VariableSyncModule&&) = delete;
    VariableSyncModule& operator=(const VariableSyncModule&) = delete;
    VariableSyncModule& operator=(VariableSyncModule&&) = delete;

private:
    class VotedSeed
    {
    public:
        VotedSeed(std::vector<Variant>&& v);
        bool takeVector(const std::vector<Variant>& v, const std::vector<Interval<double>>& limits, double distThreshold);
        VotedSeed(const VotedSeed&) = delete;
        VotedSeed& operator=(const VotedSeed&) = delete;
        VotedSeed(VotedSeed&&);
        VotedSeed& operator=(VotedSeed&&);

        std::vector<Variant> _values;
        std::vector<int> _supporterCount; // WARNING: initializer order dependency! Do not move freely!
        double _hash;
        int _totalSupCount;
    };

    // This memory resides in _store, don't delete
    ResultEntry* _ownResults;
    std::vector<std::unique_ptr<ResultEntry>> _store;
    SolverResult _publishData;
    AlicaTime _ttl4Communication;
    AlicaTime _ttl4Usage;
    AlicaEngine* _ae;
    bool _running;
    double _distThreshold;
    essentials::NotifyTimer<VariableSyncModule>* _timer;
    mutable std::mutex _mutex;
};

template <typename VarType>
int VariableSyncModule::getSeeds(const std::vector<VarType*>& query, const std::vector<Interval<double>>& limits, std::vector<Variant>& o_seeds) const
{
    const int dim = query.size();
    // TODO: use only stack memory for low dimensionality
    std::vector<Variant> vec(dim);

    std::vector<VotedSeed> seeds;

    AlicaTime earliest = _ae->getAlicaClock().now() - _ttl4Usage;
    { // for lock
        std::lock_guard<std::mutex> lock(_mutex);
        for (const std::unique_ptr<ResultEntry>& re : _store) {
            bool any = re->getValues(query, earliest, vec);
            if (!any) {
                continue;
            }
            bool found = false;
            for (VotedSeed& s : seeds) {
                if (s.takeVector(vec, limits, _distThreshold)) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                seeds.emplace_back(std::move(vec));
            }
        }
    }
#ifdef RS_DEBUG
    std::cout << "RS: Generated " << seeds.size() << "seeds" << std::endl;
    for (int i = 0; i < seeds.size(); ++i) {
        cout << "Seed " << i << ": "; // (sup:{1}): ",i);
        for (auto j = 0; j < dim; ++j) {
            cout << seeds[i].values[j] << "\t";
        }
        cout << endl;
    }
#endif

    std::sort(seeds.begin(), seeds.end(), [](const VotedSeed& a, const VotedSeed& b) {
        if (a._totalSupCount != b._totalSupCount) {
            return a._totalSupCount > b._totalSupCount;
        } else {
            return a._hash > b._hash;
        }
    });

    int maxNum = std::min(static_cast<int>(seeds.size()), dim);
    o_seeds.resize(dim * maxNum);
    for (int i = 0; i < maxNum; ++i) {
        auto sit = seeds[i]._values.begin();
        std::copy(sit, sit + dim, o_seeds.begin() + (i * dim));
    }

    return maxNum;
}

} /* namespace alica */
