/*
 * PartialAssignment.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef PARTIALASSIGNMENT_H_
#define PARTIALASSIGNMENT_H_

//#define SUCDEBUG

#include "engine/IAssignment.h"
#include "engine/Types.h"
#include "engine/collections/AssignmentCollection.h"
#include "supplementary/AgentID.h"

#include <algorithm>
#include <limits>
#include <list>
#include <math.h>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace alica
{

class EpByTaskComparer;
class EntryPoint;
class Plan;
class SuccessCollection;
class UtilityFunction;
class DynCardinality;
class SimplePlanTree;
class PartialAssignmentPool;

class PartialAssignment final : public IAssignment
{
  public:
    PartialAssignment(PartialAssignmentPool* pap);
    virtual ~PartialAssignment();
    void clear();
    static void reset(PartialAssignmentPool* pap); // has to be called before calculating the task assignment
    static PartialAssignment* getNew(PartialAssignmentPool* pap, const AgentGrp& robotIds, const Plan* plan, std::shared_ptr<SuccessCollection> sucCol);
    static PartialAssignment* getNew(PartialAssignmentPool* pap, PartialAssignment* oldPA);
    short getEntryPointCount() const override;
    int totalRobotCount() const override;
    const AgentGrp* getRobotsWorking(const EntryPoint* ep) const override;
    const AgentGrp* getRobotsWorking(int64_t epid) const override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(const EntryPoint* ep) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(int64_t epid) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(const EntryPoint* ep) override;
    bool addIfAlreadyAssigned(std::shared_ptr<SimplePlanTree> spt, const supplementary::AgentID* robot);
    bool assignRobot(const supplementary::AgentID* robotId, int index);
    std::shared_ptr<std::list<PartialAssignment*>> expand();
    bool isValid() const override;
    bool isGoal();
    static bool compareTo(PartialAssignment* thisPa, PartialAssignment* newPa);
    std::string toString() const override;
    virtual AssignmentCollection* getEpRobotsMapping() const override { return epRobotsMapping; }
    const Plan* getPlan() const { return plan; }
    std::shared_ptr<UtilityFunction> getUtilFunc();
    std::shared_ptr<SuccessCollection> getEpSuccessMapping();
    std::string assignmentCollectionToString() const override;
    std::shared_ptr<std::vector<EntryPoint*>> getEntryPoints();
    int getHash() const;
    int getHashCached() const { return _hash; }
    void setHash(int hash) const { _hash = hash; }
    bool isHashCalculated() const { return _hash != 0; }
    void setMax(double max);
    const AgentGrp& getRobotIds() const;

  private:
    PartialAssignmentPool* pap;

    // UtilityFunction
    std::shared_ptr<UtilityFunction> utilFunc;
    AssignmentCollection* epRobotsMapping;
    AgentGrp robotIds;
    std::vector<std::shared_ptr<DynCardinality>> dynCardinalities;
    const Plan* plan;
    const long PRECISION = 1073741824;
    long compareVal = 0;

    std::shared_ptr<SuccessCollection> epSuccessMapping;
    mutable int _hash;
    static EpByTaskComparer epByTaskComparer;
};

} /* namespace alica */

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
} // namespace std

#endif /* PARTIALASSIGNMENT_H_ */
