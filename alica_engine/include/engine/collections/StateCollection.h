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
    class StateAccessor {
        class Iterator {
            public:
            Iterator(int pos, const State* s, const StateCollection& col)
                : _pos(pos)
                , _s(s)
                , _col(col) {}
            bool operator!=(const iterator& o) const { return _pos != o._pos; }
            bool operator==(const iterator& o) const { return _pos == o._pos; }
            AgentIDPtr operator*() const { return _col.getAgents()[pos]; }

            Iterator& operator++() {
                do {
                    ++_pos;
                } while(pos < _col.getCount() && _col.getStates()[pos] != s);
                return *this;
            }
            private:
            int _pos;
            const State* _s;
            const StateCollection& _col;
        };
        public:
        StateAccessor(const State* s, const StateCollection& col) 
            : _s(s)
            , _col(col)
        {}
        Iterator begin() const {return Iterator(0,_s,_col);}
        Iterator end() const {return Iterator(_col.getCount(),_s,_col);}
        private:
        const State* _s;
        const StateCollection& _col;
    };

    StateCollection();
    StateCollection(const AgentSet& robotIds, const StateSet& states);
    StateCollection(int maxSize);
    StateCollection(const AssignmentCollection* ac);
    ~StateCollection();

    const AgentSet& getRobots() const { return _robotIds; }
    const StateSet& getStates() const { return _states; }

    int getCount() const { return _robotIds.size(); }

    const State* getStateOfRobot(AgentIdPtr robotId) const;

    int getRobotsInState(const State* s, AgentSet& o_robots) const;
    int getRobotsInState(int64_t sid, AgentSet& o_robots) const;
    StateAccessor getRobotsInState(const State* s) const {return StateAccessor(s,*this);}

    void getRobotsInStateSorted(const State* s, AgentSet& o_robots) const;

    std::string toString() const;

    void removeRobot(AgentIdPtr robotId);
    void clear();

    void setState(AgentIdPtr robotId, const State* state);
    void setStates(const AgentSet& robotIds, const State* state);
    void moveAllFromTo(const State* from, const State* to);

    void setInitialState(AgentIdPtr robotId, const EntryPoint* ep);
    void reconsiderOldAssignment(std::shared_ptr<Assignment> oldOne, std::shared_ptr<Assignment> newOne);

private:
    //TODO: merge the two vectors here:
    AgentSet _robotIds;
    StateSet _states;
};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
