/*
 * StateCollection.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Stefan Jakob
 */

#ifndef STATECOLLECTION_H_
#define STATECOLLECTION_H_


#include <vector>
#include <algorithm>
#include <sstream>
#include <memory>
#include <iostream>

#include "supplementary/AgentID.h"
#include "engine/Types.h"

namespace alica {

class State;
class AssignmentCollection;
class EntryPoint;
class Assignment;

class StateCollection final {
public:
    StateCollection();
    StateCollection(const AgentSet& robotIds, const StateSet& states);
    StateCollection(int maxSize);
    StateCollection(const AssignmentCollection* ac);
    ~StateCollection();

    const AgentSet& getRobots() const {return _robotIds;}
    const StateSet& getStates() const {return _states;}

    
    int getCount() const {return _robotIds.size();}

    const State* getStateOfRobot(const supplementary::AgentID* robotId) const;
    

    int getRobotsInState(const State* s, AgentSet& o_robots) const;
    int getRobotsInState(int64_t sid, AgentSet& o_robots) const;

    //TODO: fix me
    shared_ptr<vector<const supplementary::AgentID*>> getRobotsInStateSorted(const State* s) const;
    
    std::string toString() const;
    
    void removeRobot(const supplementary::AgentID* robotId);
    void clear();
    
    void setState(const supplementary::AgentID* robotId, const State* state);
    void setStates(const AgentSet& robotIds,const State* state);
    void moveAllFromTo(const State* from, const State* to);
    
    void setInitialState(const supplementary::AgentID* robotId, EntryPoint* ep);
    void reconsiderOldAssignment(std::shared_ptr<Assignment> oldOne, shared_ptr<Assignment> newOne);

private:
    AgentSet _robotIds;
    StateSet _states;
};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
