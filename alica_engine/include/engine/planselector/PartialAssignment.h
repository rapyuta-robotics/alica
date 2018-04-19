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
#include "supplementary/AgentID.h"
#include "engine/collections/AssignmentCollection.h"

#include <algorithm>
#include <limits>
#include <list>
#include <math.h>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace alica {

class EpByTaskComparer;
class EntryPoint;
class Plan;
class SuccessCollection;
class UtilityFunction;
class DynCardinality;
class SimplePlanTree;
class PartialAssignmentPool;

class PartialAssignment final : public IAssignment {
public:
    PartialAssignment(PartialAssignmentPool* pap);
    virtual ~PartialAssignment();
    void clear();
    static void reset(PartialAssignmentPool* pap);  // has to be called before calculating the task assignment
    static PartialAssignment* getNew(PartialAssignmentPool* pap, const AgentSet& robotIds, const Plan* plan,
            std::shared_ptr<SuccessCollection> sucCol);
    static PartialAssignment* getNew(PartialAssignmentPool* pap, PartialAssignment* oldPA);
    short getEntryPointCount() const override;
    int totalRobotCount();
    const AgentSet* getRobotsWorking(const EntryPoint* ep) const override;
    const AgentSet* getRobotsWorking(int64_t epid) const override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(
            const EntryPoint* ep) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(int64_t epid) override;
    std::shared_ptr<std::list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(
            const EntryPoint* ep) override;
    bool addIfAlreadyAssigned(shared_ptr<SimplePlanTree> spt, const supplementary::AgentID* robot);
    bool assignRobot(const supplementary::AgentID* robotId, int index);
    shared_ptr<list<PartialAssignment*>> expand();
    bool isValid() const override;
    bool isGoal();
    static bool compareTo(PartialAssignment* thisPa, PartialAssignment* newPa);
    std::string toString();
    AssignmentCollection* getEpRobotsMapping();
    const Plan* getPlan() const { return plan; }
    std::shared_ptr<UtilityFunction> getUtilFunc();
    std::shared_ptr<SuccessCollection> getEpSuccessMapping();
    std::string assignmentCollectionToString();
    std::shared_ptr<std::vector<EntryPoint*>> getEntryPoints();
    int getHash();
    void setHash(int hash);
    bool isHashCalculated();
    void setHashCalculated(bool hashCalculated);
    void setMax(double max);
    const AgentSet& getRobotIds() const;
    int hash = 0;  // TODO: fix me

private:
    static int pow(int x, int y);

    PartialAssignmentPool* pap;

    // UtilityFunction
    std::shared_ptr<UtilityFunction> utilFunc;
    AssignmentCollection* epRobotsMapping;
    AgentSet robotIds;
    std::vector<std::shared_ptr<DynCardinality>> dynCardinalities;
    const Plan* plan;
    const long PRECISION = 1073741824;
    long compareVal = 0;
    bool hashCalculated;

    std::shared_ptr<SuccessCollection> epSuccessMapping;
    static EpByTaskComparer epByTaskComparer;
};

} /* namespace alica */

namespace std {
template <>
struct hash<alica::PartialAssignment> {
    typedef alica::PartialAssignment argument_type;
    typedef std::size_t result_type;

    result_type operator()(argument_type& pa) const {
        if (pa.isHashCalculated()) {
            return pa.hash;
        }
        int basei = pa.getEpRobotsMapping()->getSize() + 1;
        const std::vector<const supplementary::AgentID*>* robots;
        for (int i = 0; i < pa.getEpRobotsMapping()->getSize(); ++i) {
            robots = pa.getEpRobotsMapping()->getRobots(i);
            for (const supplementary::AgentID* robot : *robots) {
                for (int idx = 0; idx < static_cast<int>(pa.getRobotIds().size()); ++idx) {
                    if (pa.getRobotIds().at(idx) == robot) {
                        pa.setHash(pa.hash + (i + 1) * pow(basei, idx));
                    }
                }
            }
        }
        pa.setHashCalculated(true);
        return pa.hash;
    }
};
}  // namespace std

#endif /* PARTIALASSIGNMENT_H_ */
