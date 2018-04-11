#pragma once

#include "supplementary/AgentID.h"
#include "engine/constraintmodul/IVariableSyncModule.h"

#include <supplementary/NotifyTimer.h>

#include <memory>
#include <vector>

namespace alica {
class Variable;
class ResultEntry;
class IAlicaCommunication;

class VariableSyncModule : public IVariableSyncModule {
public:
    VariableSyncModule(AlicaEngine* ae);
    virtual ~VariableSyncModule();

    virtual void init();
    virtual void close();
    virtual void clear();
    virtual void onSolverResult(shared_ptr<SolverResult> msg);

    void publishContent();
    virtual void postResult(long vid, shared_ptr<vector<uint8_t>>& result);
    virtual shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<uint8_t>>>>>> getSeeds(
            shared_ptr<vector<const Variable*>> query, shared_ptr<vector<shared_ptr<vector<double>>>> limits) override;

protected:
    supplementary::NotifyTimer<VariableSyncModule>* timer;
    AlicaTime ttl4Communication;
    AlicaTime ttl4Usage;

    class VotedSeed {
    public:
        VotedSeed(int dim, shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v);

        bool takeVector(shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v, vector<double>& scaling,
                double distThreshold, bool isDouble);
        shared_ptr<vector<double>> deserializeToDoubleVec(shared_ptr<vector<shared_ptr<vector<uint8_t>>>> v);
        shared_ptr<vector<shared_ptr<vector<uint8_t>>>> serializeFromDoubleVec(shared_ptr<vector<double>> d);

        shared_ptr<vector<shared_ptr<vector<uint8_t>>>> values;
        vector<int> supporterCount;
        double hash;
        int totalSupCount;
        int dim;
    };

private:
    const AlicaEngine* ae;
    const supplementary::AgentID* ownId;
    const IAlicaCommunication* communicator;
    bool running;
    bool communicationEnabled;

    vector<shared_ptr<ResultEntry>> store;
    shared_ptr<ResultEntry> ownResults;
    double distThreshold;
};
} /* namespace alica */
