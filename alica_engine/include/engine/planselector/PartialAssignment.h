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

using namespace std;
namespace alica {

class EpByTaskComparer;
class EntryPoint;
class Plan;
class SuccessCollection;
class UtilityFunction;
class DynCardinality;
class SimplePlanTree;
class PartialAssignmentPool;

class PartialAssignment : virtual public IAssignment {
public:
    PartialAssignment(PartialAssignmentPool* pap);
    virtual ~PartialAssignment();
    void clear();
    static void reset(PartialAssignmentPool* pap);  // has to be called before calculating the task assignment
    static PartialAssignment* getNew(PartialAssignmentPool* pap,
            shared_ptr<vector<const supplementary::AgentID*>> robotIds, Plan* plan,
            shared_ptr<SuccessCollection> sucCol);
    static PartialAssignment* getNew(PartialAssignmentPool* pap, PartialAssignment* oldPA);
    short getEntryPointCount();
    int totalRobotCount();
    shared_ptr<vector<const supplementary::AgentID*>> getRobotsWorking(EntryPoint* ep);
    shared_ptr<vector<const supplementary::AgentID*>> getRobotsWorking(long epid);
    shared_ptr<list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(EntryPoint* ep);
    shared_ptr<list<const supplementary::AgentID*>> getRobotsWorkingAndFinished(long epid);
    shared_ptr<list<const supplementary::AgentID*>> getUniqueRobotsWorkingAndFinished(EntryPoint* ep);
    bool addIfAlreadyAssigned(shared_ptr<SimplePlanTree> spt, const supplementary::AgentID* robot);
    bool assignRobot(const supplementary::AgentID* robotId, int index);
    shared_ptr<list<PartialAssignment*>> expand();
    bool isValid();
    bool isGoal();
    static bool compareTo(PartialAssignment* thisPa, PartialAssignment* newPa);
    string toString();
    AssignmentCollection* getEpRobotsMapping();
    Plan* getPlan();
    shared_ptr<UtilityFunction> getUtilFunc();
    shared_ptr<SuccessCollection> getEpSuccessMapping();
    string assignmentCollectionToString();
    shared_ptr<vector<EntryPoint*>> getEntryPoints();
    int getHash();
    void setHash(int hash);
    bool isHashCalculated();
    void setHashCalculated(bool hashCalculated);
    void setMax(double max);
    shared_ptr<vector<const supplementary::AgentID*>> getRobotIds();
    int hash = 0;

private:
    const int INFINIT = numeric_limits<int>::max();
    static int pow(int x, int y);

protected:
    PartialAssignmentPool* pap;
    static EpByTaskComparer epByTaskComparer;
    // UtilityFunction
    shared_ptr<UtilityFunction> utilFunc;
    AssignmentCollection* epRobotsMapping;
    shared_ptr<vector<const supplementary::AgentID*>> robotIds;
    vector<shared_ptr<DynCardinality>> dynCardinalities;
    Plan* plan;
    const long PRECISION = 1073741824;
    long compareVal = 0;
    bool hashCalculated;

    shared_ptr<SuccessCollection> epSuccessMapping;
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
        shared_ptr<vector<const supplementary::AgentID*>> robots;
        for (int i = 0; i < pa.getEpRobotsMapping()->getSize(); ++i) {
            robots = pa.getEpRobotsMapping()->getRobots(i);
            for (auto& robot : *robots) {
                for (int idx = 0; idx < pa.getRobotIds()->size(); idx++) {
                    if (pa.getRobotIds()->at(idx) == robot) {
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
