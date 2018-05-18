#pragma once

#include "engine/constraintmodul/IVariableSyncModule.h"
#include "engine/constraintmodul/ResultEntry.h"
#include "engine/containers/SolverResult.h"

#include <supplementary/AgentID.h>
#include <supplementary/NotifyTimer.h>

#include <vector>

namespace alica
{
class Variable;
class ResultEntry;
class IAlicaCommunication;

class VariableSyncModule : public IVariableSyncModule
{
  public:
    VariableSyncModule(AlicaEngine* ae);
    virtual ~VariableSyncModule();

    virtual void init() override;
    virtual void close() override;
    virtual void clear() override;
    virtual void onSolverResult(const SolverResult& msg) override;

    void publishContent();
    virtual void postResult(int64_t vid, Variant result) override;
    virtual int getSeeds(const VariableGrp& query, const std::vector<Interval<double>>& limits, std::vector<Variant>& o_seeds) const override;

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

    ResultEntry _ownResults;
    std::vector<ResultEntry> _store;
    SolverResult _publishData;
    AlicaTime _ttl4Communication;
    AlicaTime _ttl4Usage;
    const AlicaEngine* _ae;
    const IAlicaCommunication* _communicator;
    bool _running;
    double _distThreshold;
    supplementary::NotifyTimer<VariableSyncModule>* _timer;
    mutable std::mutex _mutex;
};
} /* namespace alica */
