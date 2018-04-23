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
            bool operator!=(const Iterator& o) const { return _pos != o._pos; }
            bool operator==(const Iterator& o) const { return _pos == o._pos; }
            AgentIDPtr operator*() const { return _col.getRobots()[_pos]; }

            Iterator& operator++() {
                do {
                    ++_pos;
                } while (_pos < _col.getCount() && _col.getStates()[_pos] != _s);
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
                , _col(col) {}
        Iterator begin() const { return Iterator(0, _s, _col); }
        Iterator end() const { return Iterator(_col.getCount(), _s, _col); }

    private:
        const State* _s;
        const StateCollection& _col;
    };

    StateCollection();
    StateCollection(const AgentGrp& robotIds, const StateGrp& states);
    StateCollection(int maxSize);
    StateCollection(const AssignmentCollection* ac);
    ~StateCollection();

    const AgentGrp& getRobots() const { return _robotIds; }
    const StateGrp& getStates() const { return _states; }

    int getCount() const { return _robotIds.size(); }

    const State* getStateOfRobot(AgentIDPtr robotId) const;

    int getRobotsInState(const State* s, AgentGrp& o_robots) const;
    int getRobotsInState(int64_t sid, AgentGrp& o_robots) const;
    StateAccessor getRobotsInState(const State* s) const { return StateAccessor(s, *this); }

    void getRobotsInStateSorted(const State* s, AgentGrp& o_robots) const;

    std::string toString() const;

    void removeRobot(AgentIDPtr robotId);
    void clear();

    void setState(AgentIDPtr robotId, const State* state);
    void setStates(const AgentGrp& robotIds, const State* state);
    void moveAllFromTo(const State* from, const State* to);

    void setInitialState(AgentIDPtr robotId, const EntryPoint* ep);
    void reconsiderOldAssignment(std::shared_ptr<Assignment> oldOne, std::shared_ptr<Assignment> newOne);

private:
    // TODO: merge the two vectors here:
    AgentGrp _robotIds;
    StateGrp _states;
};

} /* namespace alica */

#endif /* STATECOLLECTION_H_ */
