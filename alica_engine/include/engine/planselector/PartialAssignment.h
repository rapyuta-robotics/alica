#pragma once
#include "engine/IAssignment.h"
#include "engine/Types.h"
#include "engine/UtilityInterval.h"
#include "supplementary/AgentID.h"

#include <vector>

namespace alica
{
class PartialAssignmentPool;
class TaskAssignment;

/*
class AssignmentProblem {
    public:
    AssignmentProblem(const AgentGrp& agents, std::vector<AgentGrp>& successData)
        : _availableAgents(agents)
        , _successData(successData)
        {}
    int getTotalRobotCount() const {return _availableAgents;}
    const AgentGrp& getAvailableAgents() const {return _availableAgents;}
    private:
    AgentGrp _availableAgents;
    std::vector<AgentGrp> _successData;
};*/

class PartialAssignment final : public IAssignment
{
public:
    PartialAssignment();
    ~PartialAssignment();
    void clear();
    void prepare(const Plan* p, const TaskAssignment* problem);
    bool isValid() const;
    bool isGoal() const;
    const Plan* getPlan() const { return _plan; }
    bool addIfAlreadyAssigned(SimplePlanTree* spt, AgentIDConstPtr agent, int idx);
    bool assignUnassignedAgent(int agentIdx, int epIdx);
    UtilityInterval getUtility() const { return _utility; }
    const TaskAssignment* getProblem() const { return _problem; }
    int getAssignedAgentCount() const { return _numAssignedAgents; }

    bool expand(std::vector<PartialAssignment*>& o_container, PartialAssignmentPool* pool, const Assignment* old) const;

    static bool compare(const PartialAssignment* a, const PartialAssignment* b);

    /* short getEntryPointCount() const override;
     int totalRobotCount() const override;
     const AgentGrp* getRobotsWorking(const EntryPoint* ep) const override;
     const AgentGrp* getRobotsWorking(int64_t epid) const override;
     std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(const EntryPoint* ep) override;
     std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(int64_t epid) override;
     std::shared_ptr<std::list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) override;
     bool addIfAlreadyAssigned(std::shared_ptr<SimplePlanTree> spt, const supplementary::AgentID* robot);



     std::string toString() const override;
     virtual AssignmentCollection* getEpRobotsMapping() const override { return epRobotsMapping; }

     std::shared_ptr<UtilityFunction> getUtilFunc() const { return utilFunc; }
     virtual std::shared_ptr<SuccessCollection> getEpSuccessMapping() const override { return epSuccessMapping; }
     std::string assignmentCollectionToString() const override;
     int getHash() const;
     int getHashCached() const { return _hash; }
     void setHash(int hash) const { _hash = hash; }
     bool isHashCalculated() const { return _hash != 0; }
     void setMax(double max);
     const AgentGrp& getRobotIds() const;*/

private:
    friend std::ostream& operator<<(std::ostream& out, const PartialAssignment& a);
    const Plan* _plan;
    const TaskAssignment* _problem;
    std::vector<DynCardinality> _cardinalities;
    std::vector<int> _assignment;
    UtilityInterval _utility;
    int _numAssignedAgents;
    int _nextAgentIdx;

    // long compareVal = 0;

    // std::shared_ptr<SuccessCollection> epSuccessMapping;
    // mutable int _hash;
    // static EpByTaskComparer epByTaskComparer;

    static bool s_allowIdling;
};

std::ostream& operator<<(std::ostream& out, const PartialAssignment& a);

} /* namespace alica */
/*
namespace std
{
template <>
struct hash<const alica::PartialAssignment>
{
    typedef const alica::PartialAssignment argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type& pa) const
    {
        if (pa.isHashCalculated()) {
            return pa.getHashCached();
        }
        int hash = 0;
        int basei = pa.getEpRobotsMapping()->getSize() + 1;
        const std::vector<const supplementary::AgentID*>* robots;
        for (int i = 0; i < pa.getEpRobotsMapping()->getSize(); ++i) {
            robots = pa.getEpRobotsMapping()->getRobots(i);
            for (const supplementary::AgentID* robot : *robots) {
                for (int idx = 0; idx < static_cast<int>(pa.getRobotIds().size()); ++idx) {
                    if (pa.getRobotIds().at(idx) == robot) {
                        hash = (hash + (i + 1) * pow(basei, idx));
                    }
                }
            }
        }
        pa.setHash(hash);
        return hash;
    }
};
*/
} // namespace std
