#pragma once

#include <engine/Types.h>
#include <engine/model/Plan.h>

#include <ostream>
#include <vector>

namespace alica
{
class EntryPoint;
class Plan;

class SuccessCollection
{
public:
    SuccessCollection();
    SuccessCollection(const Plan* plan);
    ~SuccessCollection();
    int getCount() const { return _successData.size(); }
    const EntryPointGrp& getEntryPoints() const { return _plan->getEntryPoints(); }

    void setSuccess(AgentIDConstPtr robot, const EntryPoint* ep);
    void clear();
    const AgentGrp* getAgents(const EntryPoint* ep) const;
    const AgentGrp* getAgentsById(int64_t id) const;
    const AgentGrp* getAgentsByIndex(int idx) const
    {
        if (idx >= 0 && idx < static_cast<int>(_successData.size())) {
            return &_successData[idx];
        }
        return nullptr;
    }
    AgentGrp& editAgentsByIndex(int idx) { return _successData[idx]; }
    const std::vector<AgentGrp>& getRaw() const { return _successData; }

private:
    friend std::ostream& operator<<(std::ostream& out, const SuccessCollection& c);
    const Plan* _plan;
    std::vector<AgentGrp> _successData;
};

std::ostream& operator<<(std::ostream& out, const SuccessCollection& c);

} /* namespace alica */
